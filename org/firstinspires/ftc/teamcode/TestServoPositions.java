package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServoPositions (Blocks to Java)", group = "")
public class TestServoPositions extends LinearOpMode {

    private Servo LeftClamp;
    private Servo RightClamp;
    private DcMotor LinearActuator;
    
public void runOpMode() {
    
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator"); 
    waitForStart();
    
if(opModeIsActive()) {
    while(opModeIsActive()) {
    
    if (gamepad2.y == true) {
          LeftClamp.setPosition(0.7);
          //RightClamp.setPosition(0.6);
          // Clamp in
        }
        if (gamepad2.b == true) {
          LeftClamp.setPosition(0.5);
          //RightClamp.setPosition(0.7);
          // Clamp out
        }
        if (gamepad2.a == true) {
          LeftClamp.setPosition(0.3);
          RightClamp.setPosition(0.8);
          // Clamp out
        }
        if (gamepad2.x == true) {
          LeftClamp.setPosition(0.1);
          RightClamp.setPosition(1);
          // Clamp out
        }
        if (gamepad2.left_bumper){
          LinearActuator.setPower(-0.9);
          
        } else if (gamepad2.right_bumper){
          LinearActuator.setPower(0.9);

        } else {
          LinearActuator.setPower(0);

        }

        
        
    }
}

}
}
