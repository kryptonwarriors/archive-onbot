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

@Autonomous(name = "TestCamera", group = "")
public class TestCamera extends LinearOpMode {

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
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private TouchSensor LFBumper;
    private TouchSensor RFBumper;
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

    PIDController pidDrive;
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

    
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");

    Color = hardwareMap.get(ColorSensor.class, "Color");
    
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");

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
    
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imuParameters.loggingEnabled      = false;
    imu.initialize(imuParameters);
    
    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated())
    {
        sleep(50);
        idle();
    }

    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    runtime.reset();
    waitForStart();

    if (opModeIsActive()) {
      
      
      pidDrive = new PIDController(.05, 0, 0);
      pidDrive.setSetpoint(0);
      pidDrive.setOutputRange(0, 0.6);
      pidDrive.setInputRange(-90, 90);
      pidDrive.enable();
        
      EncoderPID(LEFT, 1000, 0.6);
      sleep(25000);
      
      
      targetsSkyStone.activate();
      
      LeftForward.setPower(-0.18);
      RightForward.setPower(0.18);
      LeftBack.setPower(0.18);
      RightBack.setPower(-0.18);
      
      targetVisible = false;
      while(opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) <= 18  ) {
        telemetry.addData("range", String.format("%.01f in", BackDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("RunTime", runtime.seconds());
        telemetry.update();
      }
      
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      
      runtime.reset();
      while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){ 
        for (VuforiaTrackable trackable : allTrackables) {
          if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
              targetVisible = true;
              // getUpdatedRobotLocation() will return null if no new information is available since
              // the last time that call was made, or if the trackable is not currently visible.
              OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
              if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
              }
              telemetry.addData("range", String.format("%.01f in", BackDistance.getDistance(DistanceUnit.INCH)));
              telemetry.addData("Visible Target", trackable.getName());
              telemetry.update();
              break;
          }
        }
      }

      if (!targetVisible) {
        Encoder_Function(RIGHT, 240, 0.3);
        runtime.reset();
        while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){ 
          for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                  lastLocation = robotLocationTransform;
                }
  
                telemetry.addData("Visible Target", trackable.getName());
                telemetry.update();
                break;
            }
          }
        }
      }
      
      if (!targetVisible) {
        Encoder_Function(RIGHT, 240, 0.3);
        runtime.reset();
        while (opModeIsActive() && (!(targetVisible) && runtime.seconds() < 1)){ 
          for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                  lastLocation = robotLocationTransform;
                }
  
                telemetry.addData("Visible Target", trackable.getName());
                telemetry.update();
                break;
            }
          }
        }
      }
      
      telemetry.update();
      if (targetVisible) {
        // Provide feedback as to where the robot is located (if we know).
        SkyStonePos = "";
        checkForSkystone();
        sleep(3000);
        actualAdjust();
      } 
    
      targetsSkyStone.deactivate();
      sleep(1000);
      //pick up either skystone or stone
 
    LinearActuator.setPower(-0.2);
    sleep(800);
    LinearActuator.setPower(0);
    
    LeftClamp.setPosition(0.8);
    RightClamp.setPosition(0.3);
    
    RightCascade.setPower(0.2);
    LeftCascade.setPower(0.2);
    sleep(300);
    RightCascade.setPower(0);
    LeftCascade.setPower(0);
    
    LeftForward.setPower(0.18);
     RightForward.setPower(-0.18);
      LeftBack.setPower(-0.18);
      RightBack.setPower(0.18);
      RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      targetVisible = false;
      while(opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) >= 26 || RightBack.getCurrentPosition()>400 ) {
        telemetry.addData("range", String.format("%.01f in", BackDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("RunTime", runtime.seconds());
        telemetry.update();
      }
      
      LeftForward.setPower(0);
      RightForward.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
    
    RightCascade.setPower(-0.2);
    LeftCascade.setPower(-0.2);
    sleep(400);
    RightCascade.setPower(0);
    LeftCascade.setPower(0);
    
    Encoder_Function(LEFT, 1000, 0.4);
    
    
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
    
    StopAndReset();
    while ( Math.abs(LeftForward.getCurrentPosition()) <=  Math.abs(TargetPosition)  && !isStopRequested() ) {
      
      correction = pidDrive.performPID(getAngle());
    
      if (Direction == LEFT) {
        LeftForward.setPower(Power-correction);
        LeftBack.setPower(Power-correction);
        RightForward.setPower(Power+correction);
        RightBack.setPower(Power+correction);
      }
      else if (Direction == RIGHT) {
        LeftForward.setPower(-Power+correction);
        LeftBack.setPower(-Power+correction);
        RightForward.setPower(-Power-correction);
        RightBack.setPower(-Power-correction);
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
      LeftForward.setPower(-Power);
      LeftBack.setPower(-Power);
      RightForward.setPower(-Power);
      RightBack.setPower(-Power);
    }
    else if (Direction == LTurn) {
      THRESH = TURNTHRESH;
      LeftForward.setPower(Power);
      LeftBack.setPower(Power);
      RightForward.setPower(Power);
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
      sleep(200);  
  }

private void checkForSkystone() {
  
  if (targetVisible) {
  
    
    // express position (translation) of robot in inches.
    VectorF translation = lastLocation.getTranslation();
    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
      translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
      yPos = translation.get(1)/mmPerInch;
      if (yPos >=-0.7 && yPos <= 1.1) {
          SkyStonePos = "Center";
      } 
      else if (yPos > 1.1) {
        SkyStonePos = "Left";
      } 
      else if (yPos <-0.7) {
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
      while (!(SkyStonePos == "Center")){
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
      while (!(SkyStonePos == "Center")){
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
  
  if(SkyStonePos == "Left") {
    int moveRight = ((int)Math.abs(yPos - 1.1) * 21) + 80;
    telemetry.addData("right", moveRight);
    telemetry.update();
    Encoder_Function(RIGHT, moveRight, 0.3);
  }
  else if(SkyStonePos == "Right") {
    int moveLeft = ((int)Math.abs(0.7 - yPos) * 21) + 80;
    telemetry.addData("left", moveLeft);
    telemetry.update();
    Encoder_Function(LEFT, moveLeft, 0.3);
  }
  
  //robot is at the center of the skystone
  Encoder_Function(FORWARD, 400, 0.25);
  
}

} //End of Class


