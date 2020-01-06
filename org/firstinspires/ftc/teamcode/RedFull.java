package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import java.util.List;
import android.app.Activity;
import java.util.Locale;
import android.view.View;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "RedFull", group = "")
public class RedFull extends LinearOpMode {

    private static DcMotor LeftForward = null;
    private static DcMotor LeftBack = null;
    private static DcMotor RightForward = null;
    private static DcMotor RightBack = null;
    private static DcMotor LinearActuator = null;
    private static DcMotor LeftCascade = null;
    private static DcMotor RightCascade = null;

    private static Servo LeftClamp = null;
    private static Servo LeftFoundation = null;
    private static Servo RightClamp = null;
    private static Servo RightFoundation = null;

    private ColorSensor Color;
    private DistanceSensor BackDistance;
    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private TouchSensor LFBumper;
    private TouchSensor RFBumper;
    private TouchSensor LBBumper;
    private TouchSensor RBBumper;
    private HardwareDevice webcam_1;
    private BNO055IMU   imu_1;
    private BNO055IMU   imu;


    final int FORWARD = 0;
    final int BACKWARD = 1;
    final int LEFT = 2;
    final int RIGHT = 3;
    final int UP = 4;
    final int WALL = 5;
    final int RTurn = 6;
    final int LTurn = 7;
    final int EXTEND = 8;
    final int RETRACT = 9;
    int THRESH = 15;
    final int ALL_THRESH = 15;
    final int TURNTHRESH = 30;
    String TapeColor = null;
    String SkyStonePos = "";
    double yPos = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "ATtMcMX/////AAABmb5pRDectEzdvJK1epLa9N9J1oqdQ6gzb2cBjuQ0nEhg5aIm3m+cYXYZTqSUY+v8yGl8+UiYCyyG6cSF5DpWvqqy/x/cYYrv02jiaW7mhTX4B8Pfk3TcqT+COr1Z7tQqgHect1mujWffOu7TBJ5MU03uFHDUG5+X/xrSiu7mgsl+/DOILeUhXtz/8oqJVlJ/kMbFtstisbLtjui227t77vif/T0w8ZIMjB8HsKysbrk88ueZe2Sx2aEWJpUtUca4Z4DytfS4yS46lHhOEqKwLth/xHMtCFZ1nickcitpagXXRf2wTxhKOd8T9i6fnb6v/00weiIZnxfujgWpYZIab1So+yYLPWmvVLjRKRkDuYGL";

     // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private ElapsedTime runtime = new ElapsedTime();

    PIDController pidDrive, pidDistDrive;
    double        globalAngle, correction;
    Orientation   lastAngles = new Orientation();

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private int Position = 1;

    @Override
    public void runOpMode() {

    webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraName = webcamName;

    vuforia = ClassFactory.getInstance().createVuforia(parameters);
    VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
    VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
    stoneTarget.setName("SkyStone Target");

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsSkyStone);

    stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
    final float CAMERA_VERTICAL_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 2 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


    for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    telemetry.addData(">", "Camera Done");
    telemetry.update();

    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");

    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");

    Color = hardwareMap.get(ColorSensor.class, "Color");

    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
    RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");
    LBBumper = hardwareMap.get(RevTouchSensor.class, "LBBumper");
    RBBumper = hardwareMap.get(RevTouchSensor.class, "RBBumper");
    

    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");

    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    LeftClamp.setPosition(0.7);
    RightClamp.setPosition(0.4);

    LeftFoundation.setPosition(0.8);
    RightFoundation.setPosition(0.22);

    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imuParameters.loggingEnabled      = false;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(imuParameters);


    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated())
    {
        sleep(50);
        idle();
        telemetry.addData(">", "calibrating");
        telemetry.update();
    }

    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    runtime.reset();
    waitForStart();

    if (opModeIsActive()) {

      pidDrive = new PIDController(0.05, 0, 0);
      pidDrive.setSetpoint(0);
      pidDrive.setInputRange(-90, 90);
      pidDrive.enable();

      pidDistDrive = new PIDController(0.028, 0, 0);
      pidDistDrive.setSetpoint(0);
      pidDistDrive.setInputRange(-90, 90);
      pidDistDrive.enable();


      targetsSkyStone.activate();

      DistancePID(FORWARD, 19, 0.25, 1.7);
      //resetAngle();
      telemetry.addData("Distance", BackDistance.getDistance(DistanceUnit.INCH));
      telemetry.update();
      runtime.reset();

      VuforiaTrackable lastTrackable = null;
      while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){
        for (VuforiaTrackable trackable : allTrackables) {
          if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
              targetVisible = true;
              Position = 3;
              // getUpdatedRobotLocation() will return null if no new information is available since
              // the last time that call was made, or if the trackable is not currently visible.
              OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
              if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                lastTrackable = trackable;
              }
              telemetry.addData("range", String.format("%.01f in", BackDistance.getDistance(DistanceUnit.INCH)));
              telemetry.addData("Visible Target", trackable.getName());
              telemetry.update();
              break;
          }
        }
      }

      if (!targetVisible) {
        EncoderPID(LEFT, 170, 0.3);
        runtime.reset();
        while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){
          for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                Position = 2;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                  lastLocation = robotLocationTransform;
                  lastTrackable = trackable;
                }

                telemetry.addData("Visible Target", trackable.getName());
                telemetry.update();
                break;
            }
          }
        }
      }

      if (!targetVisible) {
        EncoderPID(LEFT, 160, 0.3);
        runtime.reset();
        while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){
          for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                Position = 3; 
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                  lastLocation = robotLocationTransform;
                  lastTrackable = trackable;
                }

                telemetry.addData("Visible Target", trackable.getName());
                telemetry.update();
                break;
            }
          }
        }
      }

      if (targetVisible) {
        LinearActuator.setPower(0.8);
        sleep(500);
        LinearActuator.setPower(0);
        // Provide feedback as to where the robot is located (if we know).
        SkyStonePos = "";
        checkForSkystone();
        sleep(100);
        actualAdjust();
        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)lastTrackable.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
        SkyStonePos = "";
        checkForSkystone();
        if(!(SkyStonePos == "Center")) {
          sleep(100);
          actualAdjust2();
        }
        EncoderPID(FORWARD, 350, 0.25);
      }

      targetsSkyStone.deactivate();
    

      //Detection is done, pick up either skystone or stone

      LeftClamp.setPosition(0.9);
      RightClamp.setPosition(0.25);
      sleep(100);
      
      RightCascade.setPower(0.2);
      LeftCascade.setPower(0.2);
      sleep(250);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);

      DistancePID(BACKWARD, 23, 0.18, 18);
      resetAngle();
      LinearActuator.setPower(-0.8);
      sleep(300);
      LinearActuator.setPower(0);
      EncoderPID(RIGHT, 1500, 0.5);
      RightCascade.setPower(0.4);
      LeftCascade.setPower(0.4);
      sleep(200);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);
      
      RightForward.setPower(-0.3);
      LeftBack.setPower(-0.3);
      LeftForward.setPower(0.3);
      RightBack.setPower(0.3);
      sleep(300);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      LeftForward.setPower(0);
      RightBack.setPower(0);
      /*if (Position == 3) {
        EncoderPID(LEFT, 1200, 0.4);
      } else if (Position == 2) {
        EncoderPID(LEFT, 1500, 0.4);
      } else {
        EncoderPID(LEFT, 1700, 0.4);
      }*/
      //Go To the Foundation Wall

      DistancePID(RIGHT, 27, 0.4, 2);

      // Raise Up
      RightCascade.setPower(0.4);
      LeftCascade.setPower(0.4);
      LinearActuator.setPower(0.8);
      sleep(500);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);
      sleep(950);
      LinearActuator.setPower(0);
      
      // Go Forward
      moveUntilFrontBumper(0.25, 3);

      
      
      //Release Stone

      LeftClamp.setPosition(0.7);
      RightClamp.setPosition(0.4);
      sleep(500);
      DistancePID(BACKWARD, 27, 0.25, 0.4);
      
      //Drops Cascading Kit
      RightCascade.setPower(-0.4);
      LeftCascade.setPower(-0.4);
      LinearActuator.setPower(-0.8);
      sleep(700);
      RightCascade.setPower(0);
      LeftCascade.setPower(0);
      sleep(850);
      LinearActuator.setPower(0);
      
      //Turn 180 degrees
      Encoder_Function(RTurn, 950, 0.5);
      Encoder_Function(LEFT, 200, 0.5);

      moveUntilBackBumper(0.3);


      //Hold Foundation
      LeftFoundation.setPosition(0.28);
      RightFoundation.setPosition(0.72);
      sleep(800);

      moveUntilFrontBumper(0.4, 2.5);
      

      //Release Foundation
      LeftFoundation.setPosition(0.68);
      RightFoundation.setPosition(0.22);
      sleep(400);
      //Park
      DistanceUsingLeftDist(RIGHT, 35, 0.4, 2);
      telemetry.addData("Ready to Park Up/Down", LeftDistance.getDistance(DistanceUnit.INCH));
      telemetry.update();
      if (RightDistance.getDistance(DistanceUnit.INCH) >= 30) {
        Encoder_Function(RIGHT, 700, 0.5);
        telemetry.addData("Ready to Park", "Down/Wall");
      telemetry.update();
      } else {
        Encoder_Function(RIGHT, 350, 0.6);
        Encoder_Function(BACKWARD, 300, 0.45);
        Encoder_Function(RIGHT, 300, 0.5);
        telemetry.addData("Ready to Park", "Up/Bridge");
        telemetry.update();
      }

      

    } // end of if opmode active


  } //End of opmode

  /**
   * Get current cumulative angle rotation from last reset.
   * @return Angle in degrees. + = left, - = right from zero point.
   */
  private double getAngle()
  {
      // We experimentally determined the Z axis is the axis we want to use for heading angle.
      // We have to process the angle because the imu works in euler angles so the Z axis is
      // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
      // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

      Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

      if (deltaAngle < -180)
          deltaAngle += 360;
      else if (deltaAngle > 180)
          deltaAngle -= 360;

      globalAngle += deltaAngle;

      lastAngles = angles;

      return globalAngle;
  }

  private void StopAndReset(){
    LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

  }

  private void EncoderPID(int Direction, int TargetPosition, double Power) {

    pidDrive.setOutputRange(0, Power);
    pidDistDrive.setOutputRange(0, Power);
    StopAndReset();
    while ( Math.abs(LeftForward.getCurrentPosition()) <=  Math.abs(TargetPosition)  && !isStopRequested() ) {
      if (isStopRequested()) {
        StopDrive();
      }

      if (Direction == LEFT) {
        correction = pidDistDrive.performPID(getAngle());
        LeftForward.setPower(Power+correction);
        LeftBack.setPower(Power+correction);
        RightForward.setPower(Power-correction);
        RightBack.setPower(Power-correction);
      }
      else if (Direction == RIGHT) {
        correction = pidDistDrive.performPID(getAngle());
        LeftForward.setPower(-Power+correction);
        LeftBack.setPower(-Power+correction);
        RightForward.setPower(-Power-correction);
        RightBack.setPower(-Power-correction);
      }
      else if (Direction == FORWARD) {
        correction = pidDrive.performPID(getAngle());
        RightForward.setPower(Power+correction);
        LeftBack.setPower(Power-correction);
        LeftForward.setPower(-Power-correction);
        RightBack.setPower(-Power+correction);
      }

      telemetry.addData("Direction", Direction);
      telemetry.addData("key", "moving");
      telemetry.addData("LFCurrentPosition", LeftForward.getCurrentPosition());
      telemetry.addData("LFTargetPosition", -TargetPosition);
      telemetry.addData("RFCurrentPosition", RightForward.getCurrentPosition());
      telemetry.addData("RFTargetPosition", TargetPosition);
      telemetry.addData("LBCurrentPosition", LeftBack.getCurrentPosition());
      telemetry.addData("LBTargetPosition", TargetPosition);
      telemetry.addData("RBCurrentPosition", RightBack.getCurrentPosition());
      telemetry.addData("RBTargetPosition", -TargetPosition);
      telemetry.update();
    }

    StopDrive();

  }

  private void DistancePID(int Direction, double Distance, double Power, double failSafeTime) {
    StopAndReset();
    pidDrive.setOutputRange(0, Power);
    pidDistDrive.setOutputRange(0, Power);
    if (failSafeTime > 0) {
      runtime.reset();
    }
    if (Direction == FORWARD) {
      while (BackDistance.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)&& !isStopRequested()) {
        correction = pidDrive.performPID(getAngle());
        RightForward.setPower(Power+correction);
        LeftBack.setPower(Power-correction);
        LeftForward.setPower(-Power-correction);
        RightBack.setPower(-Power+correction);
        telemetry.addData("Direction", Direction);
        telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("key", "moving");
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
      }
    }
    else if (Direction == BACKWARD) {
      
      while ( BackDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime) && !isStopRequested()) {
       
       correction = pidDrive.performPID(getAngle());
        RightForward.setPower(-Power-correction);
        LeftBack.setPower(-Power+correction);
        LeftForward.setPower(Power+correction);
        RightBack.setPower(Power-correction);
        telemetry.addData("Direction", Direction);
        telemetry.addData("key", "moving");
        telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
      }
    }
      else if (Direction == LEFT) {
      while (LeftDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime) && !isStopRequested()) {
        correction = pidDistDrive.performPID(getAngle());
        LeftForward.setPower(Power-correction);
        LeftBack.setPower(Power-correction);
        RightForward.setPower(Power+correction);
        RightBack.setPower(Power+correction);
        telemetry.addData("Direction", Direction);
        telemetry.addData("key", "moving");
        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
        }
      }
      else if (Direction == RIGHT) {
      while ( RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime) && !isStopRequested() ) {
        correction = pidDistDrive.performPID(getAngle());
        LeftForward.setPower(-Power-correction);
        LeftBack.setPower(-Power-correction);
        RightForward.setPower(-Power+correction);
        RightBack.setPower(-Power+correction);
        telemetry.addData("Direction", Direction);
        telemetry.addData("key", "moving");
        telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
      }
    }

    StopDrive();
  }
private void DistanceUsingLeftDist(int Direction, double Distance, double Power, double failSafeTime) {
    StopAndReset();
    if (failSafeTime > 0) {
      runtime.reset();
    }
     if (Direction == LEFT) {
      while (LeftDistance.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime) && !isStopRequested()) {
        LeftForward.setPower(Power);
        LeftBack.setPower(Power);
        RightForward.setPower(Power);
        RightBack.setPower(Power);
        telemetry.addData("Direction", Direction);
        telemetry.addData("key", "moving");
        telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
        }
      }
      else if (Direction == RIGHT) {
      while ( LeftDistance.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime) && !isStopRequested() ) {
        LeftForward.setPower(-Power);
        LeftBack.setPower(-Power);
        RightForward.setPower(-Power);
        RightBack.setPower(-Power);
        telemetry.addData("Direction", Direction);
        telemetry.addData("key", "moving");
        telemetry.addData("BackDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("LFPower", LeftForward.getPower());
        telemetry.addData("RFPower", RightForward.getPower());
        telemetry.addData("LBPower", LeftBack.getPower());
        telemetry.addData("RBPower", RightBack.getPower());
        telemetry.update();
      }
    }

    StopDrive();
  }
  private void Encoder_Function(int Direction, int TargetPosition, double Power)
  {
    StopAndReset();

    THRESH = ALL_THRESH;
    if (Direction == FORWARD) {
      RightForward.setPower(Power);
      LeftBack.setPower(Power);
      LeftForward.setPower(-Power);
      RightBack.setPower(-Power);
    }
    else if (Direction == BACKWARD) {
      RightForward.setPower(-Power);
      LeftBack.setPower(-Power);
      LeftForward.setPower(Power);
      RightBack.setPower(Power);
    }
    else if (Direction == LEFT) {
      LeftForward.setPower(Power);
      LeftBack.setPower(Power);
      RightForward.setPower(Power);
      RightBack.setPower(Power);
    }
    else if (Direction == RIGHT) {
      LeftForward.setPower(-Power);
      LeftBack.setPower(-Power);
      RightForward.setPower(-Power);
      RightBack.setPower(-Power);
    }
    else if (Direction == RTurn) {
      THRESH = TURNTHRESH;
      RightForward.setPower(Power);
      LeftBack.setPower(-Power);
      LeftForward.setPower(Power);
      RightBack.setPower(-Power);
    }
    else if (Direction == LTurn) {
      THRESH = TURNTHRESH;
      RightForward.setPower(-Power);
      LeftBack.setPower(Power);
      LeftForward.setPower(-Power);
      RightBack.setPower(Power);
    }

    while ( ( (Math.abs(Math.abs(LeftForward.getCurrentPosition()) - Math.abs(TargetPosition)) > THRESH)
            )
            && !isStopRequested()
          )
    {
          telemetry.addData("Direction", Direction);
          telemetry.addData("key", "moving");
          telemetry.addData("LFCurrentPosition", LeftForward.getCurrentPosition());
          telemetry.addData("LFTargetPosition", -TargetPosition);
          telemetry.addData("RFCurrentPosition", RightForward.getCurrentPosition());
          telemetry.addData("RFTargetPosition", TargetPosition);
          telemetry.addData("LBCurrentPosition", LeftBack.getCurrentPosition());
          telemetry.addData("LBTargetPosition", TargetPosition);
          telemetry.addData("RBCurrentPosition", RightBack.getCurrentPosition());
          telemetry.addData("RBTargetPosition", -TargetPosition);
          telemetry.update();
    }

    StopDrive();

  } // End of function

  private void StopDrive()
  {
      LeftBack.setPower(0.0);
      LeftForward.setPower(0.0);
      RightForward.setPower(0.0);
      RightBack.setPower(0.0);
      telemetry.addData("Zero", "Motors stopped");
      telemetry.update();
      sleep(100);
  }

private void checkForSkystone() {

  if (targetVisible) {

    // express position (translation) of robot in inches.
    VectorF translation = lastLocation.getTranslation();
    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
      translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
      yPos = translation.get(1)/mmPerInch;
      if (yPos >=-0.75 && yPos <= 0.85) {
          SkyStonePos = "Center";
      }
      else if (yPos > 0.85) {
        SkyStonePos = "Left";
      }
      else if (yPos <-0.75) {
        SkyStonePos = "Right";
      }
    telemetry.addData("SkyStonePosition", SkyStonePos);
    telemetry.addData("yPos", yPos);
    // express the rotation of the robot in degrees.
    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
  }
  else {
          telemetry.addData("Visible Target", "none");
  }
  telemetry.update();
}

private void adjust() {
  if (SkyStonePos == "Center") {
      Encoder_Function(FORWARD, 400, 0.2);
   }
   else if (SkyStonePos == "Left") {
      LeftForward.setPower(-0.27);
      RightForward.setPower(-0.27);
      LeftBack.setPower(-0.27);
      RightBack.setPower(-0.27);
      while (!(SkyStonePos == "Center") && !isStopRequested()){
        telemetry.addData("yPos", yPos);
        telemetry.update();
      }
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      Encoder_Function(FORWARD, 400, 0.2);
    }
    else if (SkyStonePos == "Right") {
      LeftForward.setPower(0.27);
      RightForward.setPower(0.27);
      LeftBack.setPower(0.27);
      RightBack.setPower(0.27);
      while (!(SkyStonePos == "Center") && !isStopRequested()){
        telemetry.addData("yPos", yPos);
        telemetry.update();
      }
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);

      Encoder_Function(FORWARD, 400, 0.2);
    }
}

private void actualAdjust() {


  //while(!(SkyStonePos == "Center")) {
    if(SkyStonePos == "Left") {
      int moveRight = ((int)Math.abs(yPos - 0.85) * 16) + 30;
      telemetry.addData("right", moveRight);
      telemetry.update();
      EncoderPID(RIGHT, moveRight, 0.26);
    }
    else if(SkyStonePos == "Right") {
      int moveLeft = ((int)Math.abs(0.75 - yPos) * 16) + 30;
      telemetry.addData("left", moveLeft);
      telemetry.update();
      EncoderPID(LEFT, moveLeft, 0.26);
    }
  //}
  //robot is at the center of the skystone

}

private void actualAdjust2() {


  //while(!(SkyStonePos == "Center")) {
    if(SkyStonePos == "Left") {
      int moveRight = ((int)Math.abs(yPos - 0.85) * 10) + 20;
      telemetry.addData("right", moveRight);
      telemetry.update();
      Encoder_Function(RIGHT, moveRight, 0.26);
    }
    else if(SkyStonePos == "Right") {
      int moveLeft = ((int)Math.abs(0.75 - yPos) * 10) + 20;
      telemetry.addData("left", moveLeft);
      telemetry.update();
      Encoder_Function(LEFT, moveLeft, 0.26);
    }
  //}
  //robot is at the center of the skystone

}

private void resetAngle() {
  lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
  globalAngle = 0;

}
private void moveUntilFrontBumper (double Power, double failSafeTime) {
        RightForward.setPower(Power);
        LeftBack.setPower(Power);
        LeftForward.setPower(-Power);
        RightBack.setPower(-Power);
        if (failSafeTime > 0) {
          runtime.reset();
        }
        while (!(LFBumper.isPressed() || RFBumper.isPressed()) && (runtime.seconds() < failSafeTime && failSafeTime > 0) && !isStopRequested()) {
          
          telemetry.addData("LeftBumper", LFBumper.isPressed());
          telemetry.addData("RightBumper", RFBumper.isPressed());
          telemetry.update();
        }
        StopDrive();
}
private void moveUntilBackBumper (double Power) {
        RightForward.setPower(-Power);
        LeftBack.setPower(-Power);
        LeftForward.setPower(Power);
        RightBack.setPower(Power);
        while (! (LBBumper.isPressed() || RBBumper.isPressed()) && !isStopRequested() ) {
          telemetry.addData("LeftBackBumper", LBBumper.isPressed());
          telemetry.addData("RightBackBumper", RBBumper.isPressed());
          telemetry.update();
        }
        StopDrive();
}
} //End of Class


	