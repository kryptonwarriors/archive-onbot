package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
// DC MOTOR

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// NAV

import com.qualcomm.robotcore.hardware.Servo;
// SERVO

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import android.app.Activity;
import java.util.Locale;
import android.view.View;
import android.graphics.Color;
// UTIL

public class Util {

private static Servo LeftFoundation = null;
private static Servo RightFoundation = null;
// SERVO

private static DcMotor LeftForward = null;
private static DcMotor LeftBack = null;
private static DcMotor RightForward = null;
private static DcMotor RightBack = null;
private static DcMotor LinearActuator = null;
private static DcMotor LeftCascade = null;
private static DcMotor RightCascade = null;
// DC MOTORS

public Util(Servo inLeftFoundation, Servo inRightFoundation,
            DcMotor inLeftForward, DcMotor inLeftBack,
            DcMotor inRightForward, DcMotor inRightBack,
            DcMotor inLinearActuator, DcMotor inLeftCascade,
            DcMotor inRightCascade )
{
    LeftFoundation = inLeftFoundation;
    RightFoundation = inRightFoundation;
    LeftForward = inLeftForward;
    LeftBack = inLeftBack;
    RightForward = inRightForward;
    RightBack = inRightBack;
    LinearActuator = inLinearActuator;
    LeftCascade = inLeftCascade;
    RightCascade = inRightCascade;

}

public static void MoveTank( int Direction, int TargetPosition, double Power ) throws InterruptedException
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
// RESET THY ENCODERS.

LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// RUN WITHOUT THY ENCODERS

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


}

public static void ResetArm ( boolean RUN ) throws InterruptedException {
    if (RUN == true) {
    LeftFoundation.setPosition(0.80);
    RightFoundation.setPosition(0.22);
    // ARM UP & RESET
    } else {
        // NOTHING HERE.
    }
}

public static void ArmDown ( boolean RUN ) throws InterruptedException {
    if (RUN == true) {
    LeftFoundation.setPosition(0.28);
    RightFoundation.setPosition(0.72);
    // ARM DOWN
    } else {
        // NOTHING HERE.
    }
}

}
