package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevTouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import java.util.List;

@Autonomous(name = "BTwoStones (Blocks to Java)", group = "")
public class BTwoStones extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private DcMotor RightForward;
  private VuforiaSkyStone vuforiaSkyStone;
  private TfodSkyStone tfodSkyStone;
  private DistanceSensor BackDistance;
  private RevTouchSensor LFBumper;
  private RevTouchSensor RFBumper;
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
  int Rturn = 6;
  int LTurn = 7;
  
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
    vuforiaSkyStone = new VuforiaSkyStone();
    tfodSkyStone = new TfodSkyStone();
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");

    // Sample TFOD Op Mode
    // Initialize Vuforia.
    /*LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   */
    vuforiaSkyStone.initialize(
        "", // vuforiaLicenseKey
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
    tfodSkyStone.initialize(vuforiaSkyStone, 0.7F, true, true);
    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodSkyStone.activate();
    
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    waitForStart();
    
    if (opModeIsActive()) {
      
      runWithoutEncoders();
      LeftForward.setPower(-0.6);
      RightForward.setPower(0.6);
      LeftBack.setPower(-0.6);
      RightBack.setPower(0.6);
      
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
      
      Encoder_Function(LEFT, 900, 0.6);
      
      LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      runWithoutEncoders();
      
      LeftForward.setPower(-0.2);
      RightForward.setPower(-0.2);
      LeftBack.setPower(0.2);
      RightBack.setPower(0.2);
      
      SkystoneCenter = 1000;
      DetectSkystone(SkystoneCenter);
      
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);
      
      //TODO: Go To Foundation and Drop the Skystone 
      int move = Math.abs(LeftForward.getCurrentPosition());
      Encoder_Function(LEFT, 4500 + move, 0.7);
      
      runWithoutEncoders();
      LeftForward.setPower(-0.3);        //forward to foundation
      RightForward.setPower(0.3);
      LeftBack.setPower(-0.3);
      RightBack.setPower(0.3);
      
      while (!(LFBumper.isPressed() || RFBumper.isPressed())) {
          telemetry.addData("Digital Touch", "Is Not Pressed");
          telemetry.update();
        } 
      
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(1000);
      
      Encoder_Function(BACKWARD, 500, 0.6);
      Encoder_Function(RIGHT, 6000 + move, 0.6);
      
      Encoder_Function(FORWARD, 500, 0.6);
      Encoder_Function(BACKWARD, 500, 0.6);
      Encoder_Function(LEFT, 6000 + move, 0.6);
      Encoder_Function(RIGHT, 1300, 0.6);
      
      tfodSkyStone.deactivate();
      vuforiaSkyStone.close();
      tfodSkyStone.close();
      
    }
  }

  
  private void DetectSkystone(double SkystoneCenter) {
    
    while (SkystoneCenter > 280 && opModeIsActive()) {
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
    while ((Math.abs(Math.abs(CurrentPosition) - Math.abs(TargetPosition)) > 15) && !(isStopRequested())) {
        
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
  
}
