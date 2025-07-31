package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "ClassBotV2Java")
public class classBotV2Java extends LinearOpMode {
    private IMU imu;
    private DcMotor arm;
    private DcMotor right_drive;
    private  DcMotor left_drive;
    private ColorSensor color;
    private TouchSensor touch;
    private Servo arm_servo;

    int gyroOffset;
    double error;
    double targetAngle;
    double calculated_speed;

    int K_P;

    double P;

    private void updateGyro() {
        if (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) > 160) {
            gyroOffset += imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            imu.resetYaw();
        }
    }



    @Override
    public void runOpMode() {

    left_drive = hardwareMap.get(DcMotor.class, "right_drive");
    right_drive = hardwareMap.get(DcMotor.class, "left_drive");
    left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    imu = hardwareMap.get(IMU.class, "imu");

    targetAngle = 0;
    error = 0;
    K_P = 80;
    calculated_speed = 0;

/*
    var target_ang;
    var speed;

    while
    {
        target_ang = gamepad.get()
        correction = pid.calculate(target_ang, imu.get())

        speed = gampad.get()

        if(correction < 0)
        {
            left_motor.power(speed - abs(correction));
            right_motor.power(speed + abs(correction));
        }
        else
        {
            left_motor.power(speed + abs(correction));
            right_motor.power(speed - abs(correction));
        }
    }
 */


    waitForStart();
    if (opModeIsActive()){

        gyroOffset = 0;
        imu.resetYaw();

        while (opModeIsActive()){
            PID pid = new PID(80, 5, 5);

            updateGyro();
            error = targetAngle - GetGyro();


            telemetry.update();
            telemetry.addData("Status", "Running");
            telemetry.addData("Velocity", 3000*gamepad1.left_trigger);
            telemetry.addData("P:", P);
            telemetry.addData("error:", error);
            telemetry.addData("imu:", GetGyro());
            telemetry.addData("target angle:", targetAngle);

            if (gamepad1.left_trigger-gamepad1.right_trigger != 0){
                targetAngle = targetAngle + ( gamepad1.left_stick_x) *3;

                calculated_speed = (3000*(gamepad1.left_trigger-gamepad1.right_trigger))+pid.get(error);
                ((DcMotorEx) right_drive).setVelocity(calculated_speed);
                ((DcMotorEx) left_drive).setVelocity(calculated_speed);

            } else if (gamepad1.right_trigger-gamepad1.left_trigger == 0) {
                ((DcMotorEx) right_drive).setVelocity(0);
                ((DcMotorEx) left_drive).setVelocity(0);


            }


        }
    }
    }

// Jsi nejlepsi - Davidek <3

    private double GetGyro() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + gyroOffset;
    }
}

