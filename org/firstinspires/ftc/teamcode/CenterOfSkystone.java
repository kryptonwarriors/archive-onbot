package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name = "CenterOfSkystone", group = "")
public class CenterOfSkystone extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private DcMotor RightForward;
  private VuforiaSkyStone vuforiaSkyStone;
  private TfodSkyStone tfodSkyStone;
  private DistanceSensor Distance1;
  private DistanceSensor Distance2;
  private double eyes;
  private double boxRightEdge;
  private double boxWidth;
  private double boxLeftEdge;
  private double SkystoneCenter;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    vuforiaSkyStone = new VuforiaSkyStone();
    tfodSkyStone = new TfodSkyStone();
    Distance1 = hardwareMap.get(DistanceSensor.class, "Distance1");
    Distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");

    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    tfodSkyStone.activate();

    telemetry.addData(">", "Press Play to start");
    telemetry.update();

    // Wait for start command from Driver Station.
    waitForStart();


    if (opModeIsActive()) {
      //set power to move forward
      LeftForward.setPower(-0.7);
      RightForward.setPower(0.7);
      LeftBack.setPower(-0.7);
      RightBack.setPower(0.7);

      eyes = Distance1.getDistance(DistanceUnit.INCH);

      //move until 16 inches from the alliance wall
      while (eyes < 15) {
        eyes = Distance1.getDistance(DistanceUnit.INCH);
        telemetry.addData("> Distance ( INCH )", Double.parseDouble(JavaUtil.formatNumber(eyes, 2)));
        telemetry.update();
      }

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      LeftForward.setPower(0.5);
      RightForward.setPower(0.5);
      LeftBack.setPower(-0.5);
      RightBack.setPower(-0.5);
      sleep(800);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      LeftForward.setPower(-0.1);       //right to skystone
      RightForward.setPower(-0.1);
      LeftBack.setPower(0.1);
      RightBack.setPower(0.1);

      SkystoneCenter = 1000;
      DetectSkystone(SkystoneCenter);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      //TODO: Go To Foundation and Drop the Skystone
      GoToFoundation();
      telemetry.addData("good1","good1");
      telemetry.update();
      sleep(200);
      DropStone();
      telemetry.addData("good2","good2");
      telemetry.update();
      sleep(200);

      LeftForward.setPower(-0.7);        //forward to foundation
      RightForward.setPower(0.7);
      LeftBack.setPower(-0.7);
      RightBack.setPower(0.7);
      sleep(200);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(3000);


      LeftForward.setPower(0.7);        //backwards
      RightForward.setPower(-0.7);
      LeftBack.setPower(0.7);
      RightBack.setPower(-0.7);
      sleep(900);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      LeftForward.setPower(-0.6);     //left
      RightForward.setPower(-0.6);
      LeftBack.setPower(0.6);
      RightBack.setPower(0.6);
      sleep(1300);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      LeftForward.setPower(-0.7);        //forward
      RightForward.setPower(0.7);
      LeftBack.setPower(-0.7);
      RightBack.setPower(0.7);
      sleep(700);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);

      LeftForward.setPower(-0.6);     //left
      RightForward.setPower(-0.6);
      LeftBack.setPower(0.6);
      RightBack.setPower(0.6);
      sleep(1200);

      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      sleep(200);


      tfodSkyStone.deactivate();
      vuforiaSkyStone.close();
      tfodSkyStone.close();

    }
  }


  private void GoToFoundation()
  {
    LeftForward.setPower(0.5);
    RightForward.setPower(0.5);
    LeftBack.setPower(-0.5);
    RightBack.setPower(-0.5);


    double eyes2 = Distance2.getDistance(DistanceUnit.INCH);

    while (eyes2 > 20) {
        eyes2 = Distance2.getDistance(DistanceUnit.INCH);
        telemetry.addData("> Distance ( INCH )", Double.parseDouble(JavaUtil.formatNumber(eyes, 2)));
        telemetry.update();
    }

    LeftForward.setPower(0);
    RightForward.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
  }

  private void DropStone()
  {
    //cougars
  }

  private void DetectSkystone(double SkystoneCenter) {

    while (SkystoneCenter > 300 && opModeIsActive()) {
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

  private void displayInfo( int i, Recognition recog) {
    telemetry.addData("label " + i, recog.getLabel());
    telemetry.addData("Top Left" + i, recog.getLeft());
    telemetry.addData("Lower Right" + i, recog.getRight());
    telemetry.addData("Box Width" + i, recog.getWidth());
    telemetry.addData("Box Height" + i, recog.getHeight());
    telemetry.addData("Image Width" + i, recog.getImageWidth());
    telemetry.addData("Image Height" + i, recog.getImageHeight());
  }



}
