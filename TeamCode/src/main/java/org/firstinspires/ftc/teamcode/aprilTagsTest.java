package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Simple AprilTag Reader", group = "Vision")
public class aprilTagsTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME  = "Webcam 1";

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {


        // 1) AprilTag processor + anotace pro lepší „co vidí“ náhled
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)            // vykreslí ID do náhledu
                .setDrawAxes(true)             // vykreslí RGB osy
                .setDrawCubeProjection(true)   // vykreslí 3D krabičku
                .build();

        // 2) VisionPortal – MJPEG stream je rychlejší pro DS náhled
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG); // rychlejší náhled na DS během INIT

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        } else {
            // Pokud chceš phone camera (jen na RC telefonu), přidej:
            // builder.setCamera(BuiltinCameraName.create(BuiltinCameraDirection.BACK));
        }

        visionPortal = builder.build();

        telemetry.addLine("INIT: Na Driver Hubu je Camera Stream s anotacemi. Míř na AprilTag…");
        telemetry.update();

        // BĚHEM INIT JE VIDĚT NA DS NÁHLED S RÁMEČKY/ID
        while (opModeInInit()) {
            dumpDetections();
            telemetry.update();
            idle();
        }

        waitForStart();

        // PO STARTU DS NÁHLED NEZOBRAZUJE; zůstane textová telemetrie
        while (opModeIsActive()) {
            dumpDetections();

            // volitelné: ovládání streamu (šetření CPU)
            if (gamepad1.a) visionPortal.resumeStreaming();
            if (gamepad1.b) visionPortal.stopStreaming();

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    @SuppressLint("DefaultLocale")
    private double dumpDetections() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Detections", detections.size());

        int i = 0;
        for (AprilTagDetection d : detections) {
            telemetry.addLine(String.format("#%d  ID=%d", i, d.id));
            if (d.ftcPose != null) {
                telemetry.addData("  X (cm)", "%.1f", d.ftcPose.x);
                telemetry.addData("  Y (cm)", "%.1f", d.ftcPose.y);
                telemetry.addData("  Z (cm)", "%.1f", d.ftcPose.z);
                telemetry.addData("  Yaw", "%.1f", d.ftcPose.yaw);
                telemetry.addData("  Pitch", "%.1f", d.ftcPose.pitch);
                telemetry.addData("  Roll", "%.1f", d.ftcPose.roll);
                telemetry.addData("  Range (cm)", "%.1f", d.ftcPose.range);
                telemetry.addData("  Bearing (deg)", "%.1f", d.ftcPose.bearing);
                telemetry.addData("  Elevation (deg)", "%.1f", d.ftcPose.elevation);
                return d.ftcPose.yaw;
            } else {
                telemetry.addLine("  (Pose neni dostupny – zkontroluj velikost v knihovne tagu)");
            }
            i++;
        }
        return 0;
    }
}