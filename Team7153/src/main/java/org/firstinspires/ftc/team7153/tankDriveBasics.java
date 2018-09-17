package org.firstinspires.ftc.team7153;

//These imports import specific functions that our code will use
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
/*
* This unfinished code is meant to be used by the junior robotics coders to learn how to create a functioning tank drive
* and from there potentially create an arcade drive, mechanum drive, or any other.
* The code that is commented out is code that WILL give an error if not commented and means that it must be fixed/set to a value.
*/
@TeleOp(name="tankDriveBasics")
public class tankDriveBasics extends OpMode{
    @Override
    public void init() {
        //This creates the DcMotor object and gives it a value of null
	DcMotor  frontRight        = null;
        DcMotor  frontLeft         = null;
        DcMotor  backRight         = null;
        DcMotor  backLeft          = null;
        
        //This sets our object equal to the motor on our robot that the phone calls "frontRight"
        frontRight = HardwareMap.get(DcMotor.class, "frontRight";        
	frontLeft = HardwareMap.get(DcMotor.class, "frontLeft");	
	backRight = HardwareMap.get(DcMotor.class, "backRight");    
	backLeft = HardwareMap.get(DcMotor.class, "backLeft");    
	    
        //This sets the direction that positive values (0<x<1] will make the motor go
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
	backRight.setDirection(DcMotor.Direction.REVERSE);	
	backLeft.setDirection(DcMotor.Direction.FORWARD);			     
				
        //This sets the mode that out motor runs in. This isn't needed until you actually use the encoders
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);	
	backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);	
	backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			     
        
        //This sets the starting power of the motor to 0. The power can be in a range including and between -1 to 1
        frontRight.setPower(0);
    }

    @Override
    public void loop() {
        //This returns the value of the gamepad sticks
        
        gamepad1.left_stick_y;
        gamepad1.left_stick_x;
        gamepad1.right_stick_y;
        gamepad1.right_stick_x;
        */

        //This sets the power of the motors to whatever is in the corresponding motors .setPower()
	    
        frontLeft.setPower();
	frontRight.setPower();
	backLeft.setPower();
	backRight.setPower();
        

        //This outputs the value of the gamepad sticks to the phone for the user to see mid-operation
	    telemetry.addData("Gamepad1 Left Stick Y: ", gamepad1.left_stick_y);
	    telemetry.update();
    }
}
