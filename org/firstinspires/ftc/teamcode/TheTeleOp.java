package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TheTeleOp", group = "")
public class TheTeleOp extends LinearOpMode {

  private DcMotor RightForward, RightBack, LeftForward, LeftBack, RightCascade, LeftCascade, LinearActuator;
  private Servo LeftClamp, RightClamp, LeftFoundation, RightFoundation;
  private ElapsedTime runtime = new ElapsedTime();
  private double Multiplier = 0.7;
  private double StrafeMultiplier = 0.8;
  private Util util;

 private double Scale (double Input) {
      double Output = Input * Math.abs(Input);
      return Output;
 }

  @Override
  public void runOpMode() {

    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");

    util = new Util( LeftFoundation, RightFoundation,
                    LeftClamp, RightClamp, LeftForward,
                    LeftBack, RightForward, RightBack,
                    LinearActuator, LeftCascade, RightCascade,
                    IMU, Color, BackDistance, RBBumper, RFBumper,
                    LBBumper, LFBumper);
    util.MotorBRAKE();

    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    util.StopTank();
    telemetry.addData(">", "INIT DONE");
    // RESET TIME
    runtime.reset();
    telemetry.update();

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
       if(Math.abs(gamepad1.right_trigger) > 0.01){
         // Strafing to the Right
        LeftForward.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        LeftBack.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightForward.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightBack.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
       } else if(Math.abs(gamepad1.left_trigger) > 0.01){
        // Strafing to the Left
        LeftForward.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        LeftBack.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightForward.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightBack.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
      } else if (gamepad1.x) {
         RightBack.setPower(-0.2 * Scale(gamepad1.right_stick_y));
        RightForward.setPower(-0.2 * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(-0.2 * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(-0.2 * Scale(gamepad1.left_stick_y));
      } else if (gamepad1.a) {
         RightBack.setPower(-0.4);
        RightForward.setPower(-0.4);
        LeftForward.setPower(0.4);
        LeftBack.setPower(0.4);
      }else if (gamepad1.y) {
        RightBack.setPower(0.4);
        RightForward.setPower(0.4);
        LeftForward.setPower(-0.4);
        LeftBack.setPower(-0.4);
      } else if (gamepad1.right_bumper) {
          RightBack.setPower(-0.8);
        RightForward.setPower(-0.8);
        LeftForward.setPower(-0.8);
        LeftBack.setPower(-0.8);
      } else if (gamepad1.left_bumper) {
         RightBack.setPower(0.8);
        RightForward.setPower(0.8);
        LeftForward.setPower(0.8);
        LeftBack.setPower(0.8);
      } else {
        RightBack.setPower(-Multiplier * Scale(gamepad1.right_stick_y));
        RightForward.setPower(-Multiplier * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));
      }
      if (gamepad2.a == true) {
        util.PickStone();
      }
      if (gamepad2.b == true) {
        util.DropStone();
      }
      if (gamepad2.x == true) {
          util.ArmDown();
          // Down
        }
        if (gamepad2.y == true) {
          util.ArmUp();
        }
        if (gamepad2.left_bumper == true) {
          LeftCascade.setPower(-0.2);
          RightCascade.setPower(0.2);
          sleep(100);
          LeftCascade.setPower(0);
          RightCascade.setPower(0);
        }
        if (gamepad2.right_bumper == true) {
          LeftClamp.setPosition(1);
          RightClamp.setPosition(0);
        }
        LinearActuator.setPower(gamepad2.right_stick_y * -0.8);
        RightCascade.setPower(gamepad2.left_stick_y * -0.5);
        LeftCascade.setPower(gamepad2.left_stick_y * -0.5);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.update();
      }
    }
  }
}
