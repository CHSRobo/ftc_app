package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;	
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
	
 @TeleOp(name="MechByrd")	
public class MechByrd extends OpMode{	
	private HardwareByrd robot = new HardwareByrd();	
    @Override	
    public void init() {	
	robot.init(hardwareMap);	
    }	
    @Override	
    public void loop() {
		double maxSpeed = .4;//Defines what fraction of speed the robot will run at
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

		robot.frontLeft.setPower(v1*maxSpeed);
		robot.frontRight.setPower(v2*maxSpeed);
		robot.backLeft.setPower(v3*maxSpeed);
		robot.backRight.setPower(v4*maxSpeed);

    	if(Math.abs(gamepad2.left_stick_y)>.05){// && (-gamepad2.left_stick_y>0 || robot.lift.getCurrentPosition()>0)){
    	    robot.lift.setPower(-gamepad2.left_stick_y);
        } else {
    	    robot.lift.setPower(0);
        }

        telemetry.addData("////Gamepad", " Info////");
        telemetry.addData("//Gamepad", " 1//");
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("//Gamepad", " 2//");
        telemetry.addData("Left Stick Y", gamepad2.left_stick_y);
        telemetry.addData("////Sensors", "////");
        telemetry.addData("Gyroscope Heading", robot.imu.getQuaternionOrientation());
        telemetry.addData("////Motor", " Info////");
        telemetry.addData("FrontLeft Power: ", robot.frontLeft.getPower());
        telemetry.addData("FrontRight Power: ", robot.frontRight.getPower());
        telemetry.addData("BackLeft Power: ", robot.backLeft.getPower());
        telemetry.addData("BackRight Power: ", robot.backRight.getPower());
    	telemetry.addData("Lift Position: ", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Power: ", robot.lift.getPower());
        telemetry.update();
    }	
}