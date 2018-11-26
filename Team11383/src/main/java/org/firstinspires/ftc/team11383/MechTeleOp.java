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

@TeleOp(name = "OmniTeleOp")

public class OmniTeleOp extends OpMode{
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
	  	double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
	  
      if(gamepad1.right_bumper){robotAngle=Math.PI*7/4; r=1;}
	    else if (gamepad1.left_bumper){robotAngle=Math.PI*3/4; r=1;}
	    
      double rightX = gamepad1.right_stick_x;
	    
      final double v1 = r * Math.sqrt(2) * Math.cos(robotAngle) + rightX;
	    final double v2 = r * Math.sqrt(2) * Math.sin(robotAngle) - rightX;
	    final double v3 = r * Math.sqrt(2) * Math.sin(robotAngle) + rightX;
	    final double v4 = r * Math.sqrt(2) * Math.cos(robotAngle) - rightX;

	    robot.leftFrontMotor.setPower(v1*maxSpeed);
	  	robot.rightFrontMotor.setPower(v2*maxSpeed);
	  	robot.leftBackMotor.setPower(v3*maxSpeed);
	  	robot.rightBackMotor.setPower(v4*maxSpeed);


        if (gamepad2.dpad_up) { // pusher angle a - highest //
            robot.push.setPosition(robot.PUSH_A);
            
        } else if (gamepad2.dpad_right) { // pusher angle d - lowest //
            robot.push.setPosition(robot.PUSH_D);

        } else if (gamepad2.dpad_left) { // pusher angle b - high //
            robot.push.setPosition(robot.PUSH_B);

        } else if (gamepad2.dpad_down) { // pusher angle c - low //
            robot.push.setPosition(robot.PUSH_C);
        }
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
© 2018 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
