package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.lang.annotation.Target;
import com.qualcomm.hardware.rev.RevTouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import java.util.List;

@Autonomous(name = "RStone (Blocks to Java)", group = "")
public class RStone extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private DcMotor RightForward;
  private DcMotor LinearActuator;
  private DcMotor LeftCascade;
  private DcMotor RightCascade;
  private VuforiaSkyStone vuforiaSkyStone;
  private TfodSkyStone tfodSkyStone;
  private DistanceSensor BackDistance;
  private RevTouchSensor LFBumper;
  private RevTouchSensor RFBumper;
  private Servo LeftFoundation;
  private Servo RightFoundation;
  private Servo LeftClamp;
  private Servo RightClamp;
  private double eyes;
  private double boxRightEdge;
  private double boxWidth;
  private double boxLeftEdge;
  private double SkystoneCenter;
  int FORWARD = 0;
  int BACKWARD = 1;
  int LEFT = 2;
  int RIGHT = 3;
  int UP = 4;
  int WALL = 5;
  int park = UP;
  int RTurn = 6;
  int LTurn = 7;
  int EXTEND = 8;
  int RETRACT = 9;
  int THRESH = 18;
  int ALL_THRESH = 18;
  int TURNTHRESH = 30;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    //varun is the best, better than rahul, but moni is obviously superior
    //dhriti is the lead of the team
    //aman is the greatest
    //muthu and aarav are eh

    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");
    vuforiaSkyStone = new VuforiaSkyStone();
    tfodSkyStone = new TfodSkyStone();
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");
    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");


    vuforiaSkyStone.initialize(
        "Aep6rHH/////AAABme4+LfHFHUG+rWg5ePEH2PhmJqjgZILzDPD4pxfckKin9iIq+waMwELsjAS6l/M0nM3U1uaktfb3oAx9njXP/KlwvaExjQcQxkFil6zbo6X4uTwvwTjLgZRuBOdcc9XlBtZwsi9x4XHMQRm/29dqHdj6vQFeEhcSkd5L+UePPjnrRULYSWkKz7tatdLmMTSm70s7A7Eii4T+7evwoLwXnkvhGSo3x5gp5LaMWPFuVZsxWnazqB1zXFY2XjASeSk9bXiZsd/knmpqz7IsmuO0oaYczP+2AgB8M+GLizjcaPAXw/u/eziOjsQC1C/ZeOkOqH1wvjFX2jHPM7+l+i4sM/riBT1yMpqoB7h5zf6HLRQN", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        true, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    // Set min confidence threshold to 0.7
    tfodSkyStone.initialize(vuforiaSkyStone, 0.75F, true, true);
    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodSkyStone.activate();

    LeftFoundation.setPosition(0.68);
    RightFoundation.setPosition(0.22);
    LeftClamp.setPosition(1);
    RightClamp.setPosition(0.8);
    telemetry.addData("Distance", BackDistance.getDistance(DistanceUnit.INCH));
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    waitForStart();

    if (opModeIsActive()) {

      LeftCascade.setPower(0.2);
      RightCascade.setPower(-0.2);
      sleep(100);
      LeftCascade.setPower(0);
      RightCascade.setPower(0);

      runWithoutEncoders();
      LeftForward.setPower(-0.7);
      RightForward.setPower(0.7);
      LeftBack.setPower(-0.7);
      RightBack.setPower(0.7);

      double eyes = BackDistance.getDistance(DistanceUnit.INCH);

      //move until 15 inches from the alliance wall
      while (eyes < 15) {
        eyes = BackDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("> Distance ( INCH )", Double.parseDouble(JavaUtil.formatNumber(eyes, 2)));
        telemetry.update();
      }

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      Encoder_Function(RIGHT, 900, 0.6);

      LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      runWithoutEncoders();

      LeftForward.setPower(0.2);     //strafe right to find skystone
      RightForward.setPower(0.2);
      LeftBack.setPower(-0.2);
      RightBack.setPower(-0.2);

      SkystoneCenter = 0;
      DetectSkystone(SkystoneCenter);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      int move = Math.abs(LeftForward.getCurrentPosition());
      telemetry.addData("move", move);
      telemetry.update();

      tfodSkyStone.deactivate();
      vuforiaSkyStone.close();
      tfodSkyStone.close();



      Encoder_Function(FORWARD, 600, 0.5);
      LeftClamp.setPosition(0.8);
      RightClamp.setPosition(1);
      sleep(600);
      runWithoutEncoders();
      LinearActuator.setPower(0.4);
      sleep(1000);
      LinearActuator.setPower(0);
      LeftClamp.setPosition(1);
      RightClamp.setPosition(0.7);
      sleep(1000);
      LeftCascade.setPower(-0.2);
      RightCascade.setPower(0.15);
      sleep(300);
      LeftCascade.setPower(0);
      RightCascade.setPower(0);
      Encoder_Function(BACKWARD, 1200, 0.7);

      //TODO: Go To Foundation and Drop the Skystone
      Encoder_Function(RIGHT, 4100 + move, 0.8);

      LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      runWithoutEncoders();

      RightCascade.setPower(0.5);
      LeftCascade.setPower(-0.5);
      sleep(500);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);

      LeftForward.setPower(-0.3);        //forward to foundation
      RightForward.setPower(0.3);
      LeftBack.setPower(-0.3);
      RightBack.setPower(0.3);

      while (!(LFBumper.isPressed() || RFBumper.isPressed() || Math.abs(LeftForward.getCurrentPosition()) > 1600)) {
          telemetry.addData("Digital Touch", "Is Not Pressed");
          telemetry.addData("LeftForward current position", LeftForward.getCurrentPosition());
          telemetry.update();
        }

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);

      RightCascade.setPower(-0.4);
      LeftCascade.setPower(0.4);
      sleep(200);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);

      LinearActuator.setPower(0.4);
      sleep(500);
      LinearActuator.setPower(0);

      LeftClamp.setPosition(0.8);       //drop skystone
      RightClamp.setPosition(1);
      sleep(400);

      Encoder_Function(BACKWARD, 600, 0.7);
      RightCascade.setPower(-0.4);
      LeftCascade.setPower(0.4);
      sleep(300);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);
      LinearActuator.setPower(-0.6);
      sleep(700);
      LinearActuator.setPower(0);

      Encoder_Function(RIGHT, 2300, 0.8);

      /*tfodSkyStone.deactivate();
      vuforiaSkyStone.close();
      tfodSkyStone.close();*/

    }
  }


  private void DetectSkystone(double SkystoneCenter) {

    while (SkystoneCenter < 350 && opModeIsActive()) {
      List<Recognition> recognitions = tfodSkyStone.getRecognitions();
      if (recognitions.size() == 0) {
        telemetry.addData("TFOD", "No items detected.");
      } else {
        int index = 0;
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals("Skystone")) {
              boxRightEdge = recognition.getRight();
              boxWidth = recognition.getWidth();
              boxLeftEdge = recognition.getLeft();
              SkystoneCenter = (boxRightEdge + boxLeftEdge) / 2;
              displayInfo(index, recognition);
              index = index + 1;
            }
          }
      }
      telemetry.addData("Skystone Center ", SkystoneCenter);
      telemetry.update();
    }
  }

  /**
   * Display info (using telemetry) for a recognized object.
   */
  private void displayInfo( int i, Recognition recog) {
    telemetry.addData("label " + i, recog.getLabel());
    telemetry.addData("Top Left" + i, recog.getLeft());
    telemetry.addData("Lower Right" + i, recog.getRight());
    telemetry.addData("Box Width" + i, recog.getWidth());
    telemetry.addData("Box Height" + i, recog.getHeight());
    telemetry.addData("Image Width" + i, recog.getImageWidth());
    telemetry.addData("Image Height" + i, recog.getImageHeight());
  }

  private void runWithoutEncoders() {
      LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  private void Encoder_Function(int Direction, int TargetPosition, double Power)
  {
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int RTurn = 6;
    int LTurn = 7;

    LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    THRESH = ALL_THRESH;
    if (Direction == FORWARD) {

      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(-TargetPosition);
      LeftBack.setTargetPosition(-TargetPosition);
      RightForward.setTargetPosition(TargetPosition);
      RightBack.setTargetPosition(TargetPosition);
      LeftBack.setPower(-Power);
      LeftForward.setPower(-Power);
      RightForward.setPower(Power);
      RightBack.setPower(Power);
    }
    else if (Direction == BACKWARD) {
      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(TargetPosition);
      LeftBack.setTargetPosition(TargetPosition);
      RightForward.setTargetPosition(-TargetPosition);
      RightBack.setTargetPosition(-TargetPosition);
      LeftForward.setPower(Power);
      LeftBack.setPower(Power);
      RightBack.setPower(-Power);
      RightForward.setPower(-Power);

    } else if (Direction == LEFT) {
      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(TargetPosition);
      LeftBack.setTargetPosition(-TargetPosition);
      RightForward.setTargetPosition(TargetPosition);
      RightBack.setTargetPosition(-TargetPosition);
      LeftForward.setPower(Power);
      LeftBack.setPower(-Power);
      RightForward.setPower(Power);
      RightBack.setPower(-Power);

    }
    else if (Direction == RIGHT) {
      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(-TargetPosition);
      LeftBack.setTargetPosition(TargetPosition);
      RightForward.setTargetPosition(-TargetPosition);
      RightBack.setTargetPosition(TargetPosition);
      LeftForward.setPower(-Power);
      LeftBack.setPower(Power);
      RightForward.setPower(-Power);
      RightBack.setPower(Power);
    }
    else if (Direction == RTurn) {
      THRESH = TURNTHRESH;
      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(-TargetPosition);
      LeftBack.setTargetPosition(-TargetPosition);
      RightForward.setTargetPosition(-TargetPosition);
      RightBack.setTargetPosition(-TargetPosition);
      LeftForward.setPower(-Power);
      LeftBack.setPower(-Power);
      RightForward.setPower(-Power);
      RightBack.setPower(-Power);
    }
    else if (Direction == LTurn) {
      THRESH = TURNTHRESH;
      LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftForward.setTargetPosition(TargetPosition);
      LeftBack.setTargetPosition(TargetPosition);
      RightForward.setTargetPosition(TargetPosition);
      RightBack.setTargetPosition(TargetPosition);
      LeftForward.setPower(Power);
      LeftBack.setPower(Power);
      RightForward.setPower(Power);
      RightBack.setPower(Power);
    }

    int CurrentPosition = LeftForward.getCurrentPosition();
    while ((Math.abs(Math.abs(CurrentPosition) - Math.abs(TargetPosition)) > THRESH) && !(isStopRequested())) {

        CurrentPosition = LeftForward.getCurrentPosition();

        telemetry.addData("key", "moving");
        telemetry.addData("CurrentPosition", CurrentPosition);
        telemetry.addData("TargetPosition", TargetPosition);
        telemetry.update();
      }

    LeftForward.setPower(0);
    RightForward.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
    sleep(200);

  }

  private void LinearEncoder_Function(int Direction, int TargetPosition, double Power)
  {
    int EXTEND = 8;
    int RETRACT = 9;

    LinearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LinearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    if (Direction == EXTEND) {
      LinearActuator.setTargetPosition(TargetPosition);
      LinearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LinearActuator.setPower(Power);
    }
    else if (Direction == RETRACT) {
      LinearActuator.setTargetPosition(-TargetPosition);
      LinearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LinearActuator.setPower(-Power);
    }

    int CurrentPosition = LinearActuator.getCurrentPosition();
    while ((Math.abs(Math.abs(CurrentPosition) - Math.abs(TargetPosition)) > 15) && !(isStopRequested())) {

        CurrentPosition = LinearActuator.getCurrentPosition();

        telemetry.addData("key", "Linear Actuator is being stupid");
        telemetry.addData("CurrentPosition", CurrentPosition);
        telemetry.addData("TargetPosition", TargetPosition);
        telemetry.update();
      }

    LinearActuator.setPower(0);
    sleep(200);

  }

}
