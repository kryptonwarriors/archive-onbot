package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MinimalTeleOp", group = "")
public class MinimalTeleOp extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
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

   

    RightForward.setPower(0);
    RightBack.setPower(0);
    LeftForward.setPower(0);
    LeftBack.setPower(0);

    runtime.reset();

    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if(gamepad1.x) {}
        RightBack.setPower(0.8 * gamepad1.right_stick_y);
        RightForward.setPower(0.8 * gamepad1.right_stick_y);
        LeftForward.setPower(0.8 * gamepad1.left_stick_y);
        LeftBack.setPower(0.8 * gamepad1.left_stick_y);

        // Strafing to the Left
         LeftForward.setPower(0.8 * gamepad1.left_trigger);
         LeftBack.setPower(0.8 * gamepad1.left_trigger);
         RightForward.setPower(0.8 * gamepad1.left_trigger);
         RightBack.setPower(0.8 * gamepad1.left_trigger);

         // Strafing to the Right
         LeftForward.setPower(0.8 * gamepad1.right_trigger);
         LeftBack.setPower(0.8 * gamepad1.right_trigger);
         RightForward.setPower(-0.8 * gamepad1.right_trigger);
         RightBack.setPower(-0.8 * gamepad1.right_trigger);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();


      }
    }
  }
}

