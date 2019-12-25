package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name = "LeftStrafePark", group = "")
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
    private ColorSensor Color;
    private OpticalDistanceSensor Color_OpticalDistanceSensor;
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
    String TapeColor = "null";
    double colorHSV, hue;
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
    Color = hardwareMap.get(ColorSensor.class, "Color");
    Color_OpticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("Color");

    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    telemetry.addData(">", "INIT DONE");
    telemetry.update();

    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()){
      int colorHSV = Color.argb();
      //int colorHSV = Color.argb((int) Color.alpha(), (int) Color.red(), (int) Color.green(), (int) Color.blue());
      //int hue = (int) JavaUtil.colorToHue(colorHSV);
      int hue = (int) JavaUtil.colorToHue(colorHSV);


        if (hue < 225) {
          TapeColor = "Blue";
          telemetry.addData("Color", "Blue");
          telemetry.update();
        } else {
          TapeColor = "Not Blue";
          telemetry.addData("Color", "Not Blue");
          telemetry.update();
        }

        while (TapeColor == "null") {
          LeftForward.setPower(0.1);
          RightForward.setPower(0.1);
          LeftBack.setPower(-0.1);
          RightBack.setPower(-0.1);

          // sleep(600);

          /*
          RightForward.setPower(0.2);
          RightBack.setPower(-0.2);
          LeftForward.setPower(0.2);
          LeftBack.setPower(-0.2);
          */
          telemetry.addData("TapeColor", TapeColor);
          telemetry.addData(">", "COLOR NOT DETECTED");
          telemetry.update();
        }
        RightForward.setPower(0);
        RightBack.setPower(0);
        LeftForward.setPower(0);
        LeftBack.setPower(0);

        telemetry.addData(">", "DONE");
        telemetry.update();
      }
      }
    }
  }
