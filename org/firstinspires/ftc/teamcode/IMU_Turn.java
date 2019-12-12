package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous (name = "IMU_Turn", group = "")
public class IMU_Turn extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor RightForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  private BNO055IMU imu;
  double globalAngle, power = .30, correction;
  
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
  
  @Override
  public void runOpMode() throws InterruptedException
  {
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
    
    
    
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
    
    telemetry.addData("Mode", "Calibrating...");
    telemetry.update();
    
    while (!(IMUCalibrated() && isStopRequested() != true)) {
      sleep(1000);
    }
    waitForStart();
    
    telemetry.addData("Mode", "running...");
    telemetry.update();
    
        
}
}
