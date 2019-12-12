package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

@TeleOp(name = "GodWorker", group = "")
public class GodWorker extends LinearOpMode {

  private VuforiaSkyStone vuforiaSkyStone;
  private TfodSkyStone tfodSkyStone;
  private DistanceSensor BackDistance;
  private TouchSensor LFBumper;
  private TouchSensor RFBumper;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    vuforiaSkyStone = new VuforiaSkyStone();
    tfodSkyStone = new TfodSkyStone();
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.touchSensor.get("LFBumper");
    RFBumper = hardwareMap.touchSensor.get("RFBumper");

    // Put initialization blocks here.
    vuforiaSkyStone.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        true, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    tfodSkyStone.initialize(vuforiaSkyStone, 0.6F, true, true);
    vuforiaSkyStone.activate();
    tfodSkyStone.activate();
    telemetry.addData(">", "INIT DONE.");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        eyes = BackDistance.getDistance(DistanceUnit.INCH);
        recog = tfodSkyStone.getRecognitions();
        degree = recog.estimateAngleToObject(AngleUnit.DEGREES);
        if (LFBumper.isPressed() == true) {
          telemetry.addData("Left", "PRESSED");
        } else {
          telemetry.addData("Left", "NOT PRESSED");
        }
        if (RFBumper.isPressed() == true) {
          telemetry.addData("Right", "PRESSED");
        } else {
          telemetry.addData("Right", "NOT PRESSED");
        }
        telemetry.addData("Distance Sensor Back", Double.parseDouble(JavaUtil.formatNumber(eyes, 2)));
        telemetry.addData(">", degree);
        telemetry.update();
      }
    }

    vuforiaSkyStone.close();
    tfodSkyStone.close();
  }
}

