package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "EncoderProgram (Blocks to Java)", group = "")
public class EncoderProgram extends LinearOpMode {

  private DcMotor LeftForward;
  private DcMotor RightForward;
  private DcMotor LeftBack;
  private DcMotor RightBack;
  int FORWARD = 0;
  int BACKWARD = 1;
  int LEFT = 2;
  int RIGHT = 3;
  

  /**
   * Describe this function...
   */
  private void Encoder_Function(int Direction, int TargetPosition, double Power) 
  {
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    
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


  @Override
  public void runOpMode() {
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");

    FORWARD = 0;
    BACKWARD = 1;
    LEFT = 2;
    RIGHT = 3;
    
    
    waitForStart();
    if (opModeIsActive()) {
     
     Encoder_Function(FORWARD, 1000, 0.5);
     Encoder_Function(LEFT, 1000, 0.5);
     Encoder_Function(RIGHT, 500, 0.2);
     Encoder_Function(BACKWARD, 700, 0.6);
     
    }
    
  }
}
