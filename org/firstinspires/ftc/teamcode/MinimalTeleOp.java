package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MinimalTeleOp", group = "")
public class MinimalTeleOp extends LinearOpMode {

  private DcMotor RightForward;
  private DcMotor RightBack;
  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private double Multiplier = -1.5;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");

    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    RightForward.setPower(0);
    RightBack.setPower(0);
    LeftForward.setPower(0);
    LeftBack.setPower(0);
    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {

        RightBack.setPower(-gamepad1.right_stick_y);
        RightForward.setPower(-gamepad1.right_stick_y);
        LeftForward.setPower(-gamepad1.left_stick_y);
        LeftBack.setPower(-gamepad1.left_stick_y);

        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        if(gamepad1.right_trigger > 0.01) {
          // Strafing to the Left
        LeftForward.setPower(1 * gamepad1.right_trigger);
        LeftBack.setPower(-1 * gamepad1.right_trigger);
        RightForward.setPower(-1 * gamepad1.right_trigger);
        RightBack.setPower(1 * gamepad1.right_trigger);
        
        if(gamepad1.left_trigger > 0.01) {
        // Strafing to the Right
        LeftForward.setPower(-1 * gamepad1.left_trigger);
        LeftBack.setPower(1 * gamepad1.left_trigger);
        RightForward.setPower(1 * gamepad1.left_trigger);
        RightBack.setPower(-1 * gamepad1.left_trigger);
        }
      }
    }
  }
}
