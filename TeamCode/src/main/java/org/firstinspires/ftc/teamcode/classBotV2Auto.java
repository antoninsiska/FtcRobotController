package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ClassBotV2Auto", group = "TeleOp")
public class classBotV2Auto extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private IMU imu;

    // původní proměnné nechávám
    double wheelDiameter;
    double ticksPerRev;
    double target_ticks;
    int max_speed;
    double final_speed;
    double motors_ticks;
    double motor_start_pos;
    double gear_ratio;
    double motor_pos;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Řízení přiblížení
    private static final double STOP_RANGE_CM   = 10.0;  // kde zastavit před tagem
    private static final double ALIGN_DEG       = 7.0;   // toleranční zarovnání (|bearing| < ALIGN_DEG)
    private static final double MAX_FWD_POWER   = 0.40;
    private static final double MAX_TURN_POWER  = 0.45;
    private static final double KP_TURN         = 0.02;  // power ≈ bearingDeg * KP_TURN
    private static final double KP_FWD          = 0.006; // power ≈ (range-stop) * KP_FWD

    @Override
    public void runOpMode() {
        // MOTORY (pozor: ve tvém původním kódu byly názvy stran prohozené)
        left_drive  = hardwareMap.get(DcMotor.class, "right_drive");
        right_drive = hardwareMap.get(DcMotor.class, "left_drive");

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // směr nastavíme tak, aby +power jel vpřed na obou
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // ── AprilTag LIBRARY: bez ní nedostaneš ftcPose (range/bearing)
        AprilTagLibrary.Builder lib = new AprilTagLibrary.Builder();
        double TAG_SIZE_M = 0.0508; // 2"
        // DECODE tagy (ID, label, size, jednotka) – přidej klidně další ID, co máte na poli
        lib.addTag(new AprilTagMetadata(20, "DECODE_20", TAG_SIZE_M, DistanceUnit.METER));
        lib.addTag(new AprilTagMetadata(21, "DECODE_21", TAG_SIZE_M, DistanceUnit.METER));
        lib.addTag(new AprilTagMetadata(22, "DECODE_22", TAG_SIZE_M, DistanceUnit.METER));
        lib.addTag(new AprilTagMetadata(33, "DECODE_33", TAG_SIZE_M, DistanceUnit.METER));
        lib.addTag(new AprilTagMetadata(24, "DECODE_24", TAG_SIZE_M, DistanceUnit.METER));

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(lib.build())
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // tvoje čísla (nepoužívám je v řízení, nechávám pro případné další funkce)
        wheelDiameter = 9;   // cm
        gear_ratio    = 1.6;
        ticksPerRev   = 420;
        max_speed     = 3000;

        telemetry.addLine("INIT: ukaž robotovi tag");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        sleep(300);


        approachNearestTag();

        // stop
        left_drive.setPower(0);
        right_drive.setPower(0);

        if (visionPortal != null) visionPortal.close();
    }

    private void approachNearestTag() {
        boolean aligned = false;

        while (opModeIsActive()) {
            AprilTagDetection best = nearestDetection();
            if (best == null || best.ftcPose == null) {
                // nic nevidím → toč se na místě, ať hledáš
                telemetry.addLine("Searching…");
                telemetry.update();
                continue;
            }

            double rangeCm   = best.ftcPose.range;   // cm
            double bearing   = best.ftcPose.bearing; // deg (pravá +, levá -)

            // fáze 1: zarovnání (otáčení), nejezdíme vpřed dokud není tag v zorném středu
            if (!aligned) {
                double turn = clamp(bearing * KP_TURN, -MAX_TURN_POWER, MAX_TURN_POWER);
                left_drive.setPower(turn);
                right_drive.setPower(-turn);

                if (Math.abs(bearing) <= ALIGN_DEG) aligned = true;

                telemetry.addData("Phase", "ALIGN");
                telemetry.addData("ID", best.id);
                telemetry.addData("Bearing(deg)", "%.1f", bearing);
                telemetry.addData("Turn", "%.2f", turn);
                telemetry.update();
                continue;
            }

            // fáze 2: přibližování + jemné doladění směru
            double turn = clamp(bearing * KP_TURN, -MAX_TURN_POWER, MAX_TURN_POWER);
            double err  = rangeCm - STOP_RANGE_CM;
            double fwd  = clamp(err * KP_FWD, 0.0, MAX_FWD_POWER); // NEdovolíme couvat; jen vpřed

            // když jsme moc šikmo, uber dopřednou rychlost
            if (Math.abs(bearing) > 2.0 * ALIGN_DEG) fwd *= 0.4;

            double l = fwd - turn;
            double r = fwd + turn;
            l = clamp(l, -1, 1);
            r = clamp(r, -1, 1);

            left_drive.setPower(l);
            right_drive.setPower(r);

            telemetry.addData("Phase", "APPROACH");
            telemetry.addData("ID", best.id);
            telemetry.addData("Range(cm)", "%.1f", rangeCm);
            telemetry.addData("Bearing(deg)", "%.1f", bearing);
            telemetry.addData("Cmd L/R", "%.2f / %.2f", l, r);
            telemetry.update();

            // podmínka zastavení: jsme u cílové vzdálenosti a zhruba rovně
            if (rangeCm <= STOP_RANGE_CM + 1.0 && Math.abs(bearing) < 3.0) {
                break;
            }
        }
    }

    private void spinSearch(double power) {
        power = Math.abs(power);
        left_drive.setPower(-power);
        right_drive.setPower(power);
    }

    private AprilTagDetection nearestDetection() {
        List<AprilTagDetection> list;
        try {
            list = new ArrayList<>(aprilTag.getDetections());
        } catch (Exception e) {
            return null;
        }
        AprilTagDetection best = null;
        double bestR = Double.POSITIVE_INFINITY;
        for (AprilTagDetection d : list) {
            if (d.ftcPose == null) continue;
            if (d.ftcPose.range < bestR) {
                bestR = d.ftcPose.range;
                best = d;
            }
        }
        return best;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ── tvoje původní helpery ──────────────────────────────────────────────
    private double TicksToCm(double targetDistanceCm) {
        double wheelCircumference = Math.PI * wheelDiameter;
        return ((targetDistanceCm / wheelCircumference) * ticksPerRev * gear_ratio);
    }
    public void go_straight(int value, int speed){
        motor_start_pos = (double) (Math.abs(left_drive.getCurrentPosition()) + Math.abs(right_drive.getCurrentPosition())) /2;
        target_ticks = TicksToCm(value);
        final_speed = ((double) max_speed /100) * speed;

        while (opModeIsActive() && motor_pos <= target_ticks){
            motors_ticks = (double) (Math.abs(left_drive.getCurrentPosition()) + Math.abs(right_drive.getCurrentPosition())) /2;
            motor_pos = motors_ticks - motor_start_pos;
            telemetry.addData("pos", motors_ticks);
            telemetry.addData("Lpos", left_drive.getCurrentPosition());
            telemetry.addData("Rpos", right_drive.getCurrentPosition());
            telemetry.addData("target pos", target_ticks);
            telemetry.addData("speed", final_speed);
            ((DcMotorEx) left_drive).setVelocity(-final_speed);
            ((DcMotorEx) right_drive).setVelocity(final_speed);
            telemetry.update();
        }
        ((DcMotorEx) left_drive).setPower(-0.3);
        ((DcMotorEx) right_drive).setPower(-0.3);
    }
}