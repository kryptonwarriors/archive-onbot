package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "MotorWorker", group = "")
public class MotorWorker extends LinearOpMode {

  private DcMotor TestMotor;
  private ElapsedTime timer = new ElapsedTime();
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */


  @Override
  public void runOpMode() {
 
    
    
    /*TestMotor = hardwareMap.dcMotor.get("TestMotor");

    TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    TestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
    timer.reset();

      TestMotor.setPower(1);
      while (opModeIsActive() && timer.seconds() < 5) {
        
        telemetry.addData("Current Position", TestMotor.getCurrentPosition());
        telemetry.update();
       
      }
      TestMotor.setPower(0);
      sleep(5000); */
    }
  }

