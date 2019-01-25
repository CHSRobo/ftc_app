// MechTeleOp.java
// Driver-controlled OpMode for Mechanum wheels

package org.firstinspires.ftc.team11383;

/*
 * Frankensteined from older code by Will Archer 11/26/2018
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MechTeleOp")

public class MechTeleOp extends OpMode{
    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }


    @Override
    public void loop() {


    double maxSpeed = 1;//Defines what fraction of speed the robot will run at
    //double radGyro = (robot.gyro.getHeading() * Math.PI) / 180;
    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
    double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
	  
    if(gamepad1.right_bumper){robotAngle=Math.PI*7/4; r=1;}
	    else if (gamepad1.left_bumper){robotAngle=Math.PI*3/4; r=1;}
	    
    double rightX = -gamepad1.right_stick_x;
	    
    final double v1 = r * Math.sqrt(2) * Math.cos(robotAngle) + rightX;
    final double v2 = r * Math.sqrt(2) * Math.sin(robotAngle) - rightX;
    final double v3 = r * Math.sqrt(2) * Math.sin(robotAngle) + rightX;
    final double v4 = r * Math.sqrt(2) * Math.cos(robotAngle) - rightX;

    robot.leftFrontMotor.setPower(v1*maxSpeed);
    robot.rightFrontMotor.setPower(v2*maxSpeed);
    robot.leftBackMotor.setPower(v3*maxSpeed);
    robot.rightBackMotor.setPower(v4*maxSpeed);

	float s = gamepad2.right_stick_x; // set up Lead Screw
	robot.leftActuator.setPower(s); // extends with right, retract with left
    robot.rightActuator.setPower(s); // extends with right, retract with left

    float a = gamepad2.left_stick_y; // set up Mineral Arm
    robot.

        if (gamepad2.dpad_up) { // color sensor up //
            robot.leftArm.setPosition(0);
            robot.rightArm.setPosition(0);
            
        } else if (gamepad2.dpad_down) { // color sensor down //
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(1);
        }

        if (gamepad2.a) {
            robot.hook.setPosition(1);
        } else if (gamepad2.b) {
            robot.hook.setPosition(0);
        }
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
