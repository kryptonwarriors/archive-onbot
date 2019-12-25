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


@Autonomous(name = "DELAYRightStrafePark", group = "")
public class DELAYRightStrafePark extends LinearOpMode {

    // private OpticalDistanceSensor Color_OpticalDistanceSensor;
    private ColorSensor Color;
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
    int RTurn = 6;
    int LTurn = 7;
    int EXTEND = 8;
    int RETRACT = 9;
    int THRESH = 15;
    int ALL_THRESH = 15;
    int TURNTHRESH = 30;
    int SLEEPTIME = 10; // SLEEP TIME GOES HERE.
    int SLEEP;
    String TapeColor = null;

    @Override
    public void runOpMode() {

    // Rain_OpticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("Rain");
    Color = hardwareMap.get(ColorSensor.class, "Color");
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

    SLEEP = SLEEPTIME * 1000;

    telemetry.addData(">", "INIT DONE");
    telemetry.update();
    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()){
        sleep(SLEEP); // THE TIME DEFINED ABOVE.
      int colorHSV = Color.argb();
      int hue = (int) JavaUtil.colorToHue(colorHSV);
        if (hue < 30) {
          TapeColor = "Red";
          telemetry.addData("Color", "Red");
          telemetry.update();
        } else if (hue < 225) {
          TapeColor = "Blue";
          telemetry.addData("Color", "Blue");
          telemetry.update();
        } else {
          TapeColor = "Red";
          telemetry.addData("Color", "Red");
          telemetry.update();
        }

        while (TapeColor == "Black") {
          RightForward.setPower(0.5);
          RightBack.setPower(0.5);
          LeftForward.setPower(-0.5);
          LeftBack.setPower(-0.5);
        }
        RightForward.setPower(0);
        RightBack.setPower(0);
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        telemetry.update();
      }
      }
    }
  }

