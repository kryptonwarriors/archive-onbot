package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import java.util.List;


@Autonomous(name = "LeftStrafePark (Blocks to Java)", group = "")
public class LeftStrafePark extends LinearOpMode {
  
    private DistanceSensor BackDistance;
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private TouchSensor LFBumper;
    private DcMotor LeftBack;
    private DcMotor LeftCascade;
    private Servo LeftClamp;
    private DcMotor LeftForward;
    private Servo LeftFoundation;
    private DcMotor LinearActuator;
    private TouchSensor RFBumper;
    private DcMotor RightBack;
    private DcMotor RightCascade;
    private Servo RightClamp;
    private DcMotor RightForward;
    private Servo RightFoundation;
    private HardwareDevice webcam_1;
    private Gyroscope imu_1;
    private Gyroscope imu;
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
    int THRESH = 15;
    int ALL_THRESH = 15;
    int TURNTHRESH = 30;

    @Override
    public void runOpMode() {
      
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    RightForward = hardwareMap.dcMotor.get("RightForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");
    BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
    LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
    RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");
    LeftFoundation = hardwareMap.servo.get("LeftFoundation");
    RightFoundation = hardwareMap.servo.get("RightFoundation");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");
    
    waitForStart();
    
    if (opModeIsActive()) {
    
    Encoder_Function(RIGHT, 500, 0.5);
      
      
      
      
      

            
      
      
    }
  }
  private void Encoder_Function(int Direction, int TargetPosition, double Power) {
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
      THRESH = TURNTHRESH;
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
      RightBack.setPower(0.92*Power);
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
}
