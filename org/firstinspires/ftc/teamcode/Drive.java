import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;
import  com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Drive  extends LinearOpMode{

    //DC MOTORS
    public DcMotor LeftForward = null;
    public DcMotor LeftBack = null;
    public DcMotor RightForward = null;
    public DcMotor RightBack = null;

    HardwareMap map = new HardwareMap();

    LeftForward = map.DcMotor.get("LeftForward");
    RightForward = map.DcMotor.get("RightForward");
    LeftBack = map.DcMotor.get("LeftBack");
    RightBack = map.DcMotor.get("RightBack");
    //INIT PID
    //PIDController pidDrive, pidDistDrive;
    double        globalAngle, correction;
    Orientation   lastAngles = new Orientation();

    //CONSTRUCTORS
    public Drive(DcMotor inLeftForward, DcMotor inLeftBack,
                 DcMotor inRightForward, DcMotor inRightBack) 
    {
        LeftForward = inLeftForward;
        LeftBack = inLeftBack;
        RightForward = inRightForward;
        RightBack = inRightBack;
    }

        

/*
    //TIME
    public void Forward(int time, double Power) {
        RightForward.setPower(Power);
        LeftBack.setPower(Power);
        LeftForward.setPower(-Power);
        RightBack.setPower(-Power);
        opMode.sleep(time);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        LeftForward.setPower(0);
        RightBack.setPower(0);
    }
    public void Backward(int time, double Power) {
        RightForward.setPower(-Power);
        LeftBack.setPower(-Power);
        LeftForward.setPower(Power);
        RightBack.setPower(Power);
        opMode.sleep(time);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        LeftForward.setPower(0);
        RightBack.setPower(0);
    }
    public void Left(int time, double Power) {
        RightForward.setPower(Power);
        LeftBack.setPower(Power);
        LeftForward.setPower(Power);
        RightBack.setPower(Power); 
        opMode.sleep(time);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        LeftForward.setPower(0);
        RightBack.setPower(0);
    }

    public void Right(int time, double Power) {
        RightForward.setPower(-Power);
        LeftBack.setPower(-Power);
        LeftForward.setPower(-Power);
        RightBack.setPower(-Power); 
        opMode.sleep(time);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        LeftForward.setPower(0);
        RightBack.setPower(0);
    }
    */

    public void runOpMode()
    {
        encoderDrive(4);
    }

    public void encoderDrive(int distance)
    {
        double     COUNTS_PER_MOTOR_REV    = 383.6 ;
        double     DRIVE_GEAR_REDUCTION    = 2.0 ;
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                          (WHEEL_DIAMETER_INCHES * 3.1415);
        int newLeftTarget;
        int newRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        double errorR;
        double errorL;
        double errorBR;
        double errorBL;
        double integralR = 0;
        double integralL = 0;
        double integralBr = 0;
        double integralBl = 0;
        double derivativeR = 0;
        double derivativeL = 0;
        double derivativeBr = 0;
        double derivativeBl = 0;
        double oldErrorR = 0;
        double oldErrorL = 0;
        double oldErrorBL = 0;
        double oldErrorBR = 0;
        double p_gain = 0.1;
        double i_gain = 0;
        double d_gain = 0;

        newLeftTarget = LeftForward.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newRightTarget = RightForward.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBackLeftTarget = LeftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBackRightTarget = RightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        LeftForward.setTargetPosition(newLeftTarget);
        LeftBack.setTargetPosition(newBackLeftTarget);
        RightForward.setTargetPosition(newRightTarget);
        RightBack.setTargetPosition(newBackRightTarget);

        LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        do
        {
            errorL = LeftForward.getCurrentPosition() - newLeftTarget;
            errorR = RightForward.getCurrentPosition() - newRightTarget;
            errorBL = LeftBack.getCurrentPosition() - newBackLeftTarget;
            errorBR = RightBack.getCurrentPosition() - newBackRightTarget;

            integralR += errorR;
            integralL += errorL;
            integralBl += errorBL;
            integralBr += errorBR;

            derivativeL = oldErrorL - errorL;
            derivativeR = oldErrorR - errorR;
            derivativeBl = oldErrorBR - errorBL;
            derivativeBr = oldErrorBL - errorBR;

            oldErrorR = errorR;
            oldErrorL = errorL;
            oldErrorBL = errorBL;
            oldErrorBR = errorBR;

            LeftForward.setPower((errorL * p_gain) + (integralL * i_gain) + (derivativeL * d_gain));
            RightForward.setPower((errorR * p_gain) + (integralR * i_gain) + (derivativeR * d_gain));
            LeftBack.setPower((errorBL * p_gain) + (integralBl * i_gain) + (derivativeBl * d_gain));
            RightBack.setPower((errorBR * p_gain) + (integralBr * i_gain) + (derivativeBr * d_gain));

        }while(errorR > 1 || errorL > 1 || errorBL > 1 || errorBR > 1);

    }
}


