package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorWorker", group = "")
public class MotorWorker extends LinearOpMode {

  private DcMotor RightForward;
  private DcMotor RightBack;
  private DcMotor LeftForward;
  private DcMotor LeftBack;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.a) {
          RightForward.setPower(0.8);
        } else if (gamepad1.b) {
          RightBack.setPower(0.8);
        } else if (gamepad1.x) {
          LeftForward.setPower(0.8);
        } else if (gamepad1.y) {
          LeftBack.setPower(0.8);
        } else {
          RightForward.setPower(0);
          RightBack.setPower(0);
          LeftBack.setPower(0);
          LeftForward.setPower(0);
        }
        if (gamepad1.dpad_down) {
          RightForward.setPower(-0.8);
        } else if (gamepad1.dpad_right) {
          RightBack.setPower(-0.8);
        } else if (gamepad1.dpad_left) {
          LeftForward.setPower(-0.8);
        } else if (gamepad1.dpad_right) {
          LeftBack.setPower(-0.8);
        } else {
          LeftForward.setPower(0);
          RightForward.setPower(0);
          RightBack.setPower(0);
          LeftBack.setPower(0);
        }
        RightForward.setPower(0);
        RightBack.setPower(0);
        LeftBack.setPower(0);
        LeftForward.setPower(0);
        telemetry.update();
      }
    }
  }
}
