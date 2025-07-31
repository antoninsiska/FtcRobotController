package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "MyFIRSTJavaOpMode")
@Disabled
public class MyFIRSTJavaOpMode extends LinearOpMode {

    private IMU imu;
    private DcMotor arm;
    private AndroidSoundPool androidSoundPool;
    private AndroidTextToSpeech androidTextToSpeech;
    private ColorSensor color_sensor_REV_ColorRangeSensor;
    private Servo arm_servo;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private TouchSensor touch;

    int gyroOffset;

    /**
     * Nejaka random vec pomoc nemam to rad
     */
    private void updateGyro() {
        if (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) > 160) {
            gyroOffset += imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            imu.resetYaw();
        }
    }


    @Override
    public void runOpMode() {
        double target_angle;
        int P;
        int I;
        int D;
        int I_constant;
        int D_constant;
        int p_constant;
        ElapsedTime current_time;
        NormalizedRGBA normalized_color;
        double elapsed;
        double last_time = 0;
        double last_error;
        double error = 0;
        int PID;
        double modified_joysick = 0;
        int armBottom = 0;

        imu = hardwareMap.get(IMU.class, "imu");
        arm = hardwareMap.get(DcMotor.class, "arm");
        androidSoundPool = new AndroidSoundPool();
        androidTextToSpeech = new AndroidTextToSpeech();
        color_sensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            gyroOffset = 0;
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            target_angle = 0;
            P = 0;
            I = 0;
            D = 0;
            imu.resetYaw();
            I_constant = 4;
            D_constant = 5;
            p_constant = 45;
            current_time = new ElapsedTime();
            androidSoundPool.initialize(SoundPlayer.getInstance());
            androidTextToSpeech.initialize();
            androidTextToSpeech.setLanguage("en");
            normalized_color = ((NormalizedColorSensor) color_sensor_REV_ColorRangeSensor).getNormalizedColors();
            while (opModeIsActive()) {
                // Put loop blocks here.

                telemetry.update();
                updateGyro();
                current_time.reset();
                elapsed = current_time.milliseconds();
                last_error = error;
                error = getGyro() - target_angle;
                P = (int) (error * p_constant);
                I += elapsed * error * I_constant;
                D = (int) ((error - last_error) / (error * last_error));
                PID = P + I + D;
                if (gamepad1.dpadUpWasPressed()) {
                    target_angle = 0;
                } else if (gamepad1.dpadRightWasPressed()) {
                    target_angle = 90;
                } else if (gamepad1.dpadDownWasPressed()) {
                    target_angle = 180;
                } else if (gamepad1.dpadLeftWasPressed()) {
                    target_angle = 270;
                }
                telemetry.addData("arm angle", arm.getCurrentPosition());
                arm.setPower(0);
                telemetry.addData("RightTrigger", gamepad1.right_trigger);
                telemetry.addData("servo_hand", arm_servo.getPosition());
                telemetry.addData("yaw angle", getGyro());

                if (gamepad1.psWasPressed()) {
                    androidSoundPool.play("RawRes:ss_siren");
                    right_drive.setPower(0);
                    left_drive.setPower(0);
                    while (!gamepad1.psWasPressed()) {
                        gamepad1.setLedColor(100, 0, 0, 500);
                        telemetry.addData("robot status:", "emergency stopped");
                        I = 0;
                        P = 0;
                        D = 0;
                    }
                }
                if (3000 * (gamepad1.right_trigger - gamepad1.left_trigger) != 0) {
                    telemetry.addData("status", "go");
                    ((DcMotorEx) left_drive).setVelocity(3000 * (gamepad1.right_trigger - gamepad1.left_trigger) + PID);
                    ((DcMotorEx) right_drive).setVelocity(-3000 * (gamepad1.right_trigger - gamepad1.left_trigger) + PID);
                } else if (3000 * (gamepad1.right_trigger - gamepad1.left_trigger) == 0 && gamepad1.left_stick_x != 0) {
                    telemetry.addData("status", "turn");
                    if (gamepad1.left_stick_x > 0) {
                        modified_joysick = Math.pow(Math.abs(gamepad1.left_stick_x), 3);
                    } else if (gamepad1.left_stick_x < 0) {
                        modified_joysick = Math.pow(Math.abs(gamepad1.left_stick_x), 3) * -1;
                    } else {
                    }
                    telemetry.addData("joystick", modified_joysick);
                    left_drive.setPower(modified_joysick);
                    right_drive.setPower(modified_joysick);
                    target_angle = getGyro();
                    PID = 0;
                } else {
                    telemetry.addData("status", "else go");
                    ((DcMotorEx) left_drive).setVelocity(3000 * (gamepad1.right_trigger - gamepad1.left_trigger) + PID);
                    ((DcMotorEx) right_drive).setVelocity(-3000 * (gamepad1.right_trigger - gamepad1.left_trigger) + PID);
                    I = 0;
                }
                target_angle += gamepad1.left_stick_x * -3;
                telemetry.addData("error", error);
                telemetry.addData("P", P);
                telemetry.addData("I", I);
                telemetry.addData("I", D);
                telemetry.addData("PID", PID);
                telemetry.addData("current time", current_time);
                telemetry.addData("elapsed", elapsed);
                telemetry.addData("imu", getGyro());
                telemetry.addData("target_angle", target_angle);
                telemetry.addData("bottom position", armBottom);
                telemetry.addData("color", Double.parseDouble(JavaUtil.formatNumber(normalized_color.alpha, 3)));
                if (gamepad1.right_stick_y > 0) {
                    if (touch.isPressed()) {
                        ((DcMotorEx) arm).setVelocity(0);
                        gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                        armBottom = arm.getCurrentPosition();
                    } else {
                        ((DcMotorEx) arm).setVelocity(gamepad1.right_stick_y * 300 * -1);
                        gamepad1.stopRumble();
                    }
                } else if (gamepad1.right_stick_y < 0) {
                    if (arm.getCurrentPosition() > armBottom + 200) {
                        ((DcMotorEx) arm).setVelocity(0);
                        gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    } else {
                        ((DcMotorEx) arm).setVelocity(gamepad1.right_stick_y * 300 * -1);
                        gamepad1.stopRumble();
                    }
                } else {
                    ((DcMotorEx) arm).setVelocity(0);
                    gamepad1.stopRumble();
                }
            }
        }

        androidSoundPool.close();
        androidTextToSpeech.close();
    }

    /**
     * Describe this function...
     */
    private double getGyro() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + gyroOffset;
    }
}