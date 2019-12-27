package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name = "DELAYLeftStrafeParkUp", group = "")
public class DELAYLeftStrafeParkUp extends LinearOpMode {

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
    private Gyroscope imu_1;
    private Gyroscope imu;


    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int UP = 4;
    int WALL = 5;
    int RTurn = 6;
    int LTurn = 7;
    int EXTEND = 8;
    int RETRACT = 9;
    int THRESH = 15;
    int ALL_THRESH = 15;
    int TURNTHRESH = 30;
    String TapeColor = null;

   
    @Override
    public void runOpMode() {

    
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

    telemetry.addData(">", "INIT DONE");
    telemetry.update();

    waitForStart();

    if (opModeIsActive()) {
      sleep(10000);
      Encoder_Function(FORWARD, 350, 0.5);
      Encoder_Function(LEFT, 950, 0.5);
      // Encoder_Function(BACKWARD, 1000, 0.3);
    }

  } //End of opmode

  private void Encoder_Function(int Direction, int TargetPosition, double Power)
  {
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int RTurn = 6;
    int LTurn = 7;

    LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    while ( ( (Math.abs(Math.abs(LeftBack.getCurrentPosition()) - Math.abs(TargetPosition)) > THRESH)
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

    LeftBack.setPower(0.0);
    LeftForward.setPower(0.0);
    RightForward.setPower(0.0);
    RightBack.setPower(0.0);
    telemetry.addData("Zero", "Motors stopped");
    telemetry.update();

  } // End of function

} //End of Class

