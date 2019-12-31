// import com.qualcomm.robotcore.brain.Moni;

package org.firstinspires.ftc.teamcode;

// DC MOTOR
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;


// NAV
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


// SERVO
import com.qualcomm.robotcore.hardware.Servo;


// UTIL
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import java.util.ArrayList;
import android.app.Activity;
import java.util.Locale;
import android.view.View;
import java.lang.annotation.Target;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;

// CAM
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// SENSORS
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


public class Util {

// SERVO
public static Servo LeftFoundation = null;
public static Servo RightFoundation = null;

public static Servo LeftClamp = null;
public static Servo RightClamp = null;

// DC MOTORS
public static DcMotor LeftForward = null;
public static DcMotor LeftBack = null;
public static DcMotor RightForward = null;
public static DcMotor RightBack = null;

public static DcMotor LinearActuator = null;

public static DcMotor LeftCascade = null;
public static DcMotor RightCascade = null;

// SENSORS
public static BNO055IMU IMU;
private ColorSensor Color;
private DistanceSensor BackDistance;

// BUMPERS
private TouchSensor LFBumper;
private TouchSensor RFBumper;

private TouchSensor LBBumper;
private TouchSensor RBBumper;

// MISC VARIABLES.
public static int THRESH = 15;
public static int TURNTHRESH = 30;
int ALL_THRESH = 15;
double ms;
int TargetPosition;
double Power;

public Util(Servo inLeftFoundation, Servo inRightFoundation,
            Servo inLeftClamp, Servo inRightClamp,
            DcMotor inLeftForward, DcMotor inLeftBack,
            DcMotor inRightForward, DcMotor inRightBack,
            DcMotor inLinearActuator, DcMotor inLeftCascade,
            DcMotor inRightCascade, BNO055IMU inIMU,
            ColorSensor inColor, DistanceSensor inBackDistance,
            TouchSensor inRBBumper, TouchSensor inRFBumper,
            TouchSensor inLBBumper, TouchSensor inLFBumper)
{
    // SERVO
    LeftFoundation = inLeftFoundation;
    RightFoundation = inRightFoundation;
    LeftClamp = inLeftClamp;
    RightClamp = inRightClamp;

    // DC MOTORS
    LeftForward = inLeftForward;
    LeftBack = inLeftBack;
    RightForward = inRightForward;
    RightBack = inRightBack;
    LinearActuator = inLinearActuator;
    LeftCascade = inLeftCascade;
    RightCascade = inRightCascade;

    // SENSORS
    IMU = inIMU;
    Color = inColor;
    BackDistance = inBackDistance;

    // BUMPERS
    RBBumper = inRBBumper;
    RFBumper = inRFBumper;
    LFBumper = inLFBumper;
    LBBumper = inLBBumper;

}

public static void MoveTank( int Direction, int TargetPosition, double Power ) {
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

public static void ArmUp () {
    // ARM UP & RESET REEEEE
    LeftFoundation.setPosition(0.7);
    RightFoundation.setPosition(0.22);

}

public static void ArmDown () {
    // ARM DOWN & PULL
    LeftFoundation.setPosition(0.28);
    RightFoundation.setPosition(0.72);
}

public static void PickStone () {
    // CLAMP IN & PICK UP/CLOSE
    LeftClamp.setPosition(0.8);
    RightClamp.setPosition(0.3);
}

public static void DropStone () {
  // CLAMP OUT & DROP/OPEN
    LeftClamp.setPosition(0.7);
    RightClamp.setPosition(0.4);
}

public static void Extend ( int Direction, int Power ) {
    int Retract = 0;
    int Extend = 1;

    if (Direction == Retract) {
        LinearActuator.setPower(-Power);
    } else if (Direction == Extend) {
        LinearActuator.setPower(Power);
    }
}

public static void MotorBRAKE () {
  LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}


public static void StopTank () {
  LeftBack.setPower(0.0);
  LeftForward.setPower(0.0);
  RightForward.setPower(0.0);
  RightBack.setPower(0.0)
}

/*
public static void Cascade ( int Level, int Power) {
    int reset = 0;
    int onel = 1;
    int twol = 2;
    int threel = 3;
    // int Power = 0.4;

    LeftCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // RESET THY ENCODERS.

    LeftCascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightCascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    if ( Level = reset ){
        LeftCascade.setTargetPosition(0);
        RightCascade.setTargetPosition(0);

        LeftCascade.setPower(Power);
        RightCascade.setPower(Power);
    } else if ( Level = onel ) {
        LeftCascade.setTargetPosition();
        RightCascade.setTargetPosition(0);

        LeftCascade.setPower(Power);
        RightCascade.setPower(Power);

    }
}
*/

  // BY MONI.
}

