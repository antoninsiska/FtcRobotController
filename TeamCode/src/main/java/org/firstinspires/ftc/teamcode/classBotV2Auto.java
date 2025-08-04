package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "ClassBotV2Auto", group = "Autonomous")
public class classBotV2Auto extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private IMU imu;
    double wheelDiameter;
    double ticksPerRev;
    double target_ticks;
    int max_speed;
    double final_speed;
    double motors_ticks;
    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "right_drive");
        right_drive = hardwareMap.get(DcMotor.class, "left_drive");
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();
        wheelDiameter = 9; // cm
        ticksPerRev = 420; // HD hex motor (28CPR * 15:1 = 420)
        max_speed = 3000;
        final_speed = 0;
        motors_ticks = 0;

        if (opModeIsActive()){
            go_straight(30, 50);


        }
    }
    private double TicksToCm(double targetDistanceCm) {
        double wheelCircumference = Math.PI * wheelDiameter;
        return ((targetDistanceCm / wheelCircumference) * ticksPerRev);
    }

    public void go_straight(int value, int speed){
        target_ticks = TicksToCm(value);
        final_speed = ((double) max_speed /100) * speed;

        while (motors_ticks <= target_ticks || opModeIsActive()){
            motors_ticks = Math.abs((double) (left_drive.getCurrentPosition() + right_drive.getCurrentPosition()) /2);
            telemetry.addData("pos", motors_ticks);
            telemetry.addData("speed", final_speed);
            ((DcMotorEx) left_drive).setVelocity(-final_speed);
            ((DcMotorEx) right_drive).setVelocity(final_speed);
            telemetry.update();
        }
        ((DcMotorEx) left_drive).setVelocity(0);
        ((DcMotorEx) right_drive).setVelocity(0);




    }
}
