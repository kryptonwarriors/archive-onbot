/*
package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ;
// import AMAN;
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BetterTeleOp", group = "")
public class BetterTeleOp extends LinearOpMode {

  private DcMotor RightForward;
  private DcMotor RightBack;
  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private Servo LeftFoundation;
  private Servo RightFoundation;
  private Servo LeftClamp;
  private Servo RightClamp;
  /**
  public void runOpMode() {
    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");

    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
<<<<<<< HEAD
        LeftBack.setPower(2 * gamepad1.left_stick_y);
        RightBack.setPower(-2 * gamepad1.right_stick_y);
        LeftForward.setPower(-2 * gamepad1.left_stick_y);
        RightForward.setPower(2 * gamepad1.right_stick_y);
=======
        // Legs Function.
        RightBack.setPower(Multiplier * -gamepad1.right_stick_y);
        RightForward.setPower(Multiplier * -gamepad1.right_stick_y);
        LeftForward.setPower(Multiplier * -gamepad1.left_stick_y);
        LeftBack.setPower(Multiplier * -gamepad1.left_stick_y);

        /*
>>>>>>> 6ea5fea28b87ea626284521e761fb5a52eaa21ac
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        // Strafing to the Left
        LeftForward.setPower(2 * gamepad1.right_trigger);
        LeftBack.setPower(2 * gamepad1.right_trigger);
        RightForward.setPower(2 * gamepad1.right_trigger);
        RightBack.setPower(2 * gamepad1.right_trigger);
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        // Strafing to the Right
        LeftForward.setPower(2 * gamepad1.left_trigger);
        LeftBack.setPower(-2 * gamepad1.left_trigger);
        RightForward.setPower(2 * gamepad1.left_trigger);
        RightBack.setPower(-2 * gamepad1.left_trigger);
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        */
/*
         if (gamepad1.y == true) {
          LeftForward.setPower(-0.8);
          LeftBack.setPower(-0.8);
          RightForward.setPower(0.8);
          RightBack.setPower(0.8);
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        } else if (gamepad1.a == true) {
          LeftForward.setPower(0.8);
          LeftBack.setPower(0.8);
          RightForward.setPower(-0.8);
          RightBack.setPower(-0.8);
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
        }
        if (gamepad1.dpad_up == true) {
          RightForward.setPower(0);
          RightBack.setPower(1);
          LeftForward.setPower(-1);
          LeftBack.setPower(0);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
          // Diagonal Right Up
        } else if (gamepad1.dpad_down == true) {
          RightForward.setPower(0);
          RightBack.setPower(-1);
          LeftForward.setPower(1);
          LeftBack.setPower(0);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
          // Diagonal Left Down
        } else if (gamepad1.dpad_left == true) {
          LeftForward.setPower(0);
          LeftBack.setPower(-1);
          RightForward.setPower(1);
          RightBack.setPower(0);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
          // Diagonal Left Up
        } else if (gamepad1.dpad_right == true) {
          RightForward.setPower(-1);
          RightBack.setPower(0);
          LeftForward.setPower(0);
          LeftBack.setPower(1);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
          // Diagonal Right Down
        } else if (gamepad1.left_bumper == true) {
          LeftForward.setPower(1);
          RightForward.setPower(1);
          LeftBack.setPower(1);
          RightBack.setPower(1);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
        } else if (gamepad1.right_bumper == true) {
          LeftForward.setPower(-1);
          RightForward.setPower(-1);
          LeftBack.setPower(-1);
          RightBack.setPower(-1);
          telemetry.addData("RightForward", RightForward.getPower());
          telemetry.addData("LeftForward", LeftForward.getPower());
          telemetry.addData("RightBack", RightBack.getPower());
          telemetry.addData("LeftBack", LeftBack.getPower());
          telemetry.update();
        }
        if (gamepad2.a == true) {
          //LeftClamp.setPosition(1);
          // RightClamp.setPosition(0.8);
          // Clamp in
        }
        if (gamepad2.b == true) {
          //LeftClamp.setPosition(0.8);
          //RightClamp.setPosition(1);
          // Clamp out

        }
        if (gamepad2.x == true) {
<<<<<<< HEAD
          LeftFoundation.setPosition(0.28);
          RightFoundation.setPosition(0.72);
          // Down
=======
          LeftFoundation.setPosition(0.1);
          RightFoundation.setPosition(0.80);
          // Down Servo
>>>>>>> 6ea5fea28b87ea626284521e761fb5a52eaa21ac
        }
        if (gamepad2.y == true) {
          LeftFoundation.setPosition(0.68);
          RightFoundation.setPosition(0.22);
<<<<<<< HEAD
          // Up
=======
          // Up Servo
>>>>>>> 6ea5fea28b87ea626284521e761fb5a52eaa21ac
        }
        if (gamepad2.left_bumper == true) {
          //LeftCascade.setPower(-0.2);
          //for nRightCascade.setPower(0.2);
          //sleep(100);
          //LeftCascade.setPower(0);
          //RightCascade.setPower(0);
        }
        if (gamepad2.right_bumper == true) {
          //LeftClamp.setPosition(1);
          //RightClamp.setPosition(0.6);
        }

        //  RUN WITHOUT ENCODERS FOR ALL OTHER.
          //LeftCascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          //RightCascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //LinearActuator.setPower(gamepad2.right_stick_y * -0.8);
        //RightCascade.setPower(gamepad2.left_stick_y * -0.4);
        //LeftCascade.setPower(gamepad2.left_stick_y * 0.5);
      }
    }
  }
}
*/
