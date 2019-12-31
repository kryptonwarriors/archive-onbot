package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MotorWorker", group = "")
public class MotorWorker extends LinearOpMode {

  private DcMotor RightForward;
  private DcMotor RightBack;
  private DcMotor LeftForward;
  private DcMotor LeftBack;

  private Util util;

  @Override
  public void runOpMode() {
    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");

    util = new Util( LeftFoundation, RightFoundation,
                    LeftClamp, RightClamp, LeftForward,
                    LeftBack, RightForward, RightBack,
                    LinearActuator, LeftCascade, RightCascade,
                    IMU, Color, BackDistance, RBBumper, RFBumper,
                    LBBumper, LFBumper);

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
          util.StopTank();
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
          util.StopTank();
        }
        util.StopTank();
        telemetry.update();
      }
    }
  }
}
