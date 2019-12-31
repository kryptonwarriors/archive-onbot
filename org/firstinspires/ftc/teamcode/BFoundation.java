package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevTouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "BFoundation", group = "")
public class BFoundation extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private DcMotor RightForward;
  private DistanceSensor BackDistance;
  private RevTouchSensor LFBumper;
  private RevTouchSensor RFBumper;
  private Servo LeftFoundation;
  private Servo RightFoundation;
  private double eyes;
  private double boxRightEdge;
  private double boxWidth;
  private double boxLeftEdge;
  private Util util;
  int FORWARD = 0;
  int BACKWARD = 1;
  int LEFT = 2;
  int RIGHT = 3;
  int UP = 4;
  int WALL = 5;
  int park = UP;
  int RTurn = 6;
  int LTurn = 7;

  @Override
  public void runOpMode() {

    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");
    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");

    util.ArmUp();
    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    waitForStart();

    if (opModeIsActive()) {

      util.MoveTank(RIGHT, 700, 0.6);
      util.MoveTank(BACKWARD, 2200, 0.7);
      LeftFoundation.setPosition(0);
      RightFoundation.setPosition(0.93);
      sleep(1000);
      util.MoveTank(BACKWARD, 2000, 0.7);
      LeftFoundation.setPosition(0.68);
      RightFoundation.setPosition(0.22);

      if(park == UP) {
        util.MoveTank(RIGHT, 1700, 0.6);
        util.MoveTank(FORWARD, 1500, 0.6);
        util.MoveTank(RIGHT, 1500, 0.6);
      }
      else if (park == WALL) {
        util.MoveTank(RIGHT, 3000, 0.7);
      }


    }
  }

  private void runWithoutEncoders() {
      LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

}
