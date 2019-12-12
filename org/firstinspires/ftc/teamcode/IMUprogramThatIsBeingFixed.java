package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "IMUprogramThatIsBeingFixed (Blocks to Java)", group = "")
public class IMUprogramThatIsBeingFixed extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor RightForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private BNO055IMU imu;
  int FORWARD = 0;
  int BACKWARD = 1;
  int LEFT = 2;
  int RIGHT = 3;
  double Yaw_Angle;
  double thresh = 3.0;
  

  /**
   * Describe this function...
   */
  private void IMU_Function(
      // TODO: Enter the type for argument named Direction
      int Direction, int TargetPosition, double Power) {
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
      RightForward.setPower(Power);
      RightBack.setPower(-Power);
      LeftForward.setPower(Power);
      LeftBack.setPower(-Power);
      
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
      RightForward.setPower(-Power);
      RightBack.setPower(Power);
      LeftForward.setPower(-Power);
      LeftBack.setPower(Power);
     
    }
    int CurrentPosition = LeftForward.getCurrentPosition();
    while ((Math.abs(Math.abs(CurrentPosition) - Math.abs(TargetPosition)) > 15) && !(isStopRequested())) {
        
        CurrentPosition = LeftForward.getCurrentPosition();
        
        telemetry.addData("Power", Power);
        telemetry.addData("key", "moving");
        // Update the Motors to the new powers
        telemetry.addData("CurrentPosition", CurrentPosition);
        
        telemetry.update();
        
      }
    
    LeftForward.setPower(0);
    RightForward.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
    
    Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    telemetry.addData("Yaw_Angle", Yaw_Angle);
    telemetry.addData("rounded Yaw_Angle", (int)Math.round(Yaw_Angle));
    telemetry.update();
    sleep(2000);
    
    
    LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    TargetPosition = (int)Math.round(Yaw_Angle) *25;
    LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftForward.setTargetPosition(TargetPosition);
    LeftBack.setTargetPosition(TargetPosition);
    RightForward.setTargetPosition(TargetPosition);
    RightBack.setTargetPosition(TargetPosition);
    
    Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    while (Yaw_Angle > thresh || Yaw_Angle < -thresh) {
      Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      
      
      if (Yaw_Angle < -thresh) {
          // Turn Left
          LeftForward.setPower(0.2);
          LeftBack.setPower(0.2);
          RightForward.setPower(0.3);
          RightBack.setPower(0.3);
        } else if (Yaw_Angle > thresh) {
          // Turn Right
          LeftForward.setPower(-0.3);
          LeftBack.setPower(-0.3);
          RightForward.setPower(-0.2);
          RightBack.setPower(-0.2);
        }
        
      CurrentPosition = LeftForward.getCurrentPosition();
      telemetry.addData("fixing angle", Yaw_Angle);
      telemetry.addData("CurrentPosition", CurrentPosition);
      telemetry.addData("TargetPosition", TargetPosition);
      telemetry.update();
      
    }
    
    
    LeftForward.setPower(0);
    RightForward.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
    
  }

  /**
   * Function that becomes true when gyro is calibrated and
   * reports calibration status to Driver Station in the meantime.
   */
  private boolean IMUCalibrated() {
    telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
    telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
    telemetry.addData("System Status", imu.getSystemStatus().toString());
    telemetry.update();
    return imu.isGyroCalibrated();
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    // Put initialization blocks here.
    // Create new IMU Parameters object.
    FORWARD = 0;
    BACKWARD = 1;
    LEFT = 2;
    RIGHT = 3;
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    // Set Sensor Mode of IMU
    imuParameters.mode = BNO055IMU.SensorMode.IMU;
    // Use degrees as angle unit.
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // Express acceleration as m/s^2.
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // Disable logging.
    imuParameters.loggingEnabled = false;
    // Initialize IMU.
    imu.initialize(imuParameters);
    while (!(IMUCalibrated() && isStopRequested() != true)) {
      sleep(1000);
    }
    waitForStart();
    if (opModeIsActive()) {
      IMU_Function(FORWARD, 200, 0.7);
      IMU_Function(LEFT, 2120, 0.7);
      IMU_Function(FORWARD, 200, 0.7);
      IMU_Function(LEFT, 2120, 0.7);
    }
    LeftForward.setPower(0);
    LeftBack.setPower(0);
    RightForward.setPower(0);
    RightBack.setPower(0);
  }
}
