/*
Copyright 2019 FIRST Tech Challenge Team 16367

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
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

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
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
