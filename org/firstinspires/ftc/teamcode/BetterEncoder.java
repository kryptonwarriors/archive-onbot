
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp

public class BetterEncoder extends LinearOpMode {
    private DistanceSensor backDistance;
    private ColorSensor color;
    private Blinker control_Hub;
    private Blinker expansion_Hub;
    private TouchSensor lFBumper;
    private DcMotor leftBack;
    private DcMotor leftCascade;
    private Servo leftClamp;
    private DcMotor leftForward;
    private Servo leftFoundation;
    private DcMotor linearActuator;
    private TouchSensor rFBumper;
    private DcMotor rightBack;
    private DcMotor rightCascade;
    private Servo rightClamp;
    private DcMotor rightForward;
    private Servo rightFoundation;
    private Gyroscope imu_1;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        backDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        color = hardwareMap.get(ColorSensor.class, "Color");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub = hardwareMap.get(Blinker.class, "Expansion Hub");
        lFBumper = hardwareMap.get(TouchSensor.class, "LFBumper");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        leftCascade = hardwareMap.get(DcMotor.class, "LeftCascade");
        leftClamp = hardwareMap.get(Servo.class, "LeftClamp");
        leftForward = hardwareMap.get(DcMotor.class, "LeftForward");
        leftFoundation = hardwareMap.get(Servo.class, "LeftFoundation");
        linearActuator = hardwareMap.get(DcMotor.class, "LinearActuator");
        rFBumper = hardwareMap.get(TouchSensor.class, "RFBumper");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        rightCascade = hardwareMap.get(DcMotor.class, "RightCascade");
        rightClamp = hardwareMap.get(Servo.class, "RightClamp");
        rightForward = hardwareMap.get(DcMotor.class, "RightForward");
        rightFoundation = hardwareMap.get(Servo.class, "RightFoundation");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
