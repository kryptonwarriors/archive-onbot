package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Aman", group = "")
public class Aman extends LinearOpMode {

  private DcMotor RightForward;
  private DcMotor RightBack;
  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private double Multiplier = -1.5;
  private double LeftTrigger;
  private double RightTrigger;
  
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

    RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

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
        LeftTrigger = gamepad1.left_trigger;
        RightTrigger = gamepad1.right_trigger;

        RightBack.setPower(-gamepad1.right_stick_y);
        RightForward.setPower(-gamepad1.right_stick_y);
        LeftForward.setPower(-gamepad1.left_stick_y);
        LeftBack.setPower(-gamepad1.left_stick_y);

        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.addData("LeftTrigger", LeftTrigger);
        telemetry.addData("RightTrigger", RightTrigger);
        telemetry.update();

        
          // Strafing to the Right
        if (gamepad1.right_bumper) {
        LeftForward.setPower(-1);
        LeftBack.setPower(1);
        RightForward.setPower(-1);
        RightBack.setPower(1);
        }
        // Strafing to the Left
        if (gamepad1.left_bumper) {
        LeftForward.setPower(1);
        LeftBack.setPower(-1);
        RightForward.setPower(1);
        RightBack.setPower(-1);  
        }
        
        
        
        // Strafing to the Left
        LeftForward.setPower(-1 * LeftTrigger);
        LeftBack.setPower(-1 * LeftTrigger);
        RightForward.setPower(-1 * LeftTrigger);
        RightBack.setPower(-1 * LeftTrigger);
        
      }
    }
  }
}
