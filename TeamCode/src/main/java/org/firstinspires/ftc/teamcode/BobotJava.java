package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BobotJava")
public class BobotJava extends LinearOpMode {


    private DcMotor right_drive;
    private DcMotor left_drive;
    private IMU gyro;

    private DcMotor arm;
    private Servo arm_servo;

    double modified_joysitck;
    double calculated_speed;
    double targetAngle;
    double error;
    PID pid = new PID(50, 5, 5);

    int motorPos;
    int servoPos;
    public int getMotorPos() {
        return arm.getCurrentPosition() - motorPos;
    }

    public void resetMotorPos() {
        motorPos = getMotorPos();

    }

    public int getServoPos(){
        return ((int) arm_servo.getPosition()) - servoPos;
    }

    public  void resetServoPos(){
        servoPos = ((int) arm_servo.getPosition());
    }


  public void runOpMode(){

      left_drive = hardwareMap.get(DcMotor.class, "left_drive");
      right_drive = hardwareMap.get(DcMotor.class, "right_drive");
      gyro = hardwareMap.get(IMU.class, "gyro");
      arm = hardwareMap.get(DcMotor.class, "arm");
      arm_servo = hardwareMap.get(Servo.class, "arm_servo");
      waitForStart();
      error = 0;
      targetAngle = 0;
      modified_joysitck = 0;

      while (opModeIsActive()){
          if (gamepad1.left_stick_y != 0){
              telemetry.addData("Movement status", "go");
              modified_joysitck = (Math.pow(gamepad1.left_stick_x, 3)+(0.7 * gamepad1.left_stick_x)) * 0.6;
              targetAngle = targetAngle - (modified_joysitck) *3;

              calculated_speed = (3000*(gamepad1.left_stick_y));
              ((DcMotorEx) right_drive).setVelocity(-calculated_speed-pid.get(error));
              ((DcMotorEx) left_drive).setVelocity(calculated_speed-pid.get(error));

          } else if (gamepad1.right_trigger-gamepad1.left_trigger == 0) {
              telemetry.addData("Movement status", "stop");
              ((DcMotorEx) right_drive).setVelocity(0);
              ((DcMotorEx) left_drive).setVelocity(0);


          } else if (gamepad1.right_stick_x != 0){
              telemetry.addData("Movement status", "turn");
              calculated_speed = (3000*(gamepad1.left_stick_y));
              ((DcMotorEx) right_drive).setVelocity(calculated_speed);
              ((DcMotorEx) left_drive).setVelocity(calculated_speed);

          } else {
              telemetry.addData("Movement status", "stop");
              ((DcMotorEx) right_drive).setVelocity(0);
              ((DcMotorEx) left_drive).setVelocity(0);
          }

          arm.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
          telemetry.addData("arm pos", arm.getCurrentPosition());
          telemetry.addData("arm servo", arm_servo.getPosition());
          telemetry.update();

          if (gamepad1.circle){
              resetServoPos();
              resetMotorPos();
          }

          arm_servo.setPosition(0.5);
      }
  }
}
