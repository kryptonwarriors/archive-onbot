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


@Autonomous(name = "REDFoundationPark", group = "")
public class REDFoundationPark extends LinearOpMode {

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

    private Util util;

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

    util = new Util( LeftFoundation, RightFoundation,
                    LeftClamp, RightClamp, LeftForward,
                    LeftBack, RightForward, RightBack,
                    LinearActuator, LeftCascade, RightCascade,
                    IMU, Color, BackDistance, RBBumper, RFBumper,
                    LBBumper, LFBumper);

    util.MotorBRAKE();
    util.ArmUp();

    telemetry.addData(">", "INIT DONE");
    telemetry.update();

    waitForStart();

    if (opModeIsActive()) {

      util.MoveTank(BACKWARD, 650, 0.3);
      sleep(500);
      util.MoveTank(LEFT, 250, 0.3);

      util.ArmDown();

      sleep(500);
      RightForward.setPower(0.4);
      LeftBack.setPower(0.4);
      LeftForward.setPower(-0.4);
      RightBack.setPower(-0.4);
      while (! (LFBumper.isPressed() || RFBumper.isPressed()) ) {
        telemetry.addData(">", LFBumper.isPressed());
        telemetry.addData(">", RFBumper.isPressed());
        telemetry.update();
      }
      util.StopTank();

      sleep(500);

      util.ArmUp();

      util.MoveTank(RIGHT, 700, 0.4);

      RightForward.setPower(0.3);
      LeftBack.setPower(0.3);
      LeftForward.setPower(-0.3);
      RightBack.setPower(-0.3);

      while (! (LFBumper.isPressed() || RFBumper.isPressed()) ) {
        telemetry.addData(">", "Not Touching");
        telemetry.update();
      }
      util.MoveTank(RIGHT, 300, 0.4);

      util.StopTank();

      telemetry.addData(">", "OP MODE DONE");
      telemetry.update();
    }

  } //End of opmode

} //End of Class

