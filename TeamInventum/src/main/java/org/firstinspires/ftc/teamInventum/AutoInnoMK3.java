package org.firstinspires.ftc.teamInventum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamInventum.HardwareInventum.*;


public class AutoInnoMK3 extends LinearOpMode {
	HardwareInventum robot = new HardwareInventum(); //Gets robot from HardwareByrd class
	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0

	ElapsedTime runTime = new ElapsedTime();

	//
	void autonomousInit(){

		////////////////////////////////////////////////////////////////////////////////////Hardware////////////////////////////////////////////////////////////////////////////////
		robot.init(hardwareMap);
		robot.color.enableLed(false);

		//Calibrate the Gyroscope
		telemetry.addData("Gyro", " Calibrating. Do Not move!");
		telemetry.update();
		robot.gyro.calibrate();

		while (!isStopRequested() && robot.gyro.isCalibrating()) {
			sleep(50);
			idle();
		}

		telemetry.addData("Gyro", " Calibrated.");
		telemetry.update();

	}

	void autonomousStart() throws InterruptedException{
		//Reset the gyroscope to account for drift
		robot.gyro.resetZAxisIntegrator();

		//Reset the timer to 0
		runTime.reset();

	}

	void grab(boolean grab){
		if(grab){
			robot.leftClaw.setPosition(LEFT_CLAMP_CLOSE);
			robot.rightClaw.setPosition(RIGHT_CLAMP_CLOSE);
		} else {
			robot.leftClaw.setPosition(LEFT_CLAMP_OPEN);
			robot.rightClaw.setPosition(RIGHT_CLAMP_OPEN);
		}
	}

	void move (double power, int time){
		robot.leftMotor.setPower(power);
		robot.rightMotor.setPower(power);
		sleep(time);
		stopMoving();
	}

	private void resetTimer(){
		INPUT_TIMER = runTime.milliseconds();
	}

	void stopMoving(){
		robot.leftMotor.setPower(0);
		robot.rightMotor.setPower(0);
	}

	void straighten(){
		turn(imaginaryAngle,.2);
	}

	void telemetry(){
		telemetry.clear();
		telemetry.addData("///////Sensors//////","/");
		telemetry.addData("Red Detected:  ",robot.color.red());
		telemetry.addData("Blue Detected: ",robot.color.blue());
		telemetry.addData("Gyro Value:    ",robot.gyro.getHeading());
		telemetry.addData("///////Motors//////","/");
		telemetry.addData("Left Motor:    ",robot.leftMotor.getPower());
		telemetry.addData("Right Motor:   ",robot.rightMotor.getPower());
		telemetry.update();
	}

	void turn(double angle, double speed){
		if (isStopRequested()) {
			return;
		}
		//Sets the angle that the robot is supposed to be in to the angle argument
		imaginaryAngle = angle;
		//While the angel is > the gyroscope+TURN_ERROR or < the gyroscope-TURN_ERROR
		resetTimer();
		while ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
			if (isStopRequested()) {
				stopMoving();
				return;
			}
			//Checks to see if turning left or right
			if ((angle > robot.gyro.getHeading() && angle < robot.gyro.getHeading() + 181) || (angle < robot.gyro.getHeading() - 180)) {
				robot.leftMotor.setPower(-speed);
				robot.rightMotor.setPower(speed);
			} else {
				robot.leftMotor.setPower(speed);
				robot.rightMotor.setPower(-speed);
			}
			telemetry();
		}
		stopMoving();
		if ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
			straighten();
		}
	}

	void jewelDisplacer(boolean COLOR_REMAINING){
		robot.hammer.setPosition(HAMMER_DOWN);
		robot.color.enableLed(true);
		sleep(1000);
		if(robot.color.blue()<robot.color.red()){
			if(COLOR_REMAINING == RED){move(.1,350);}else{move(-.1,500);}
		} else if (robot.color.blue()>robot.color.red()){
			if(COLOR_REMAINING == BLUE){move(.1,350);}else{move(-.1,500);}
		}
		robot.hammer.setPosition(HAMMER_UP);
		robot.color.enableLed(false);
		sleep(1000);
	}


	@Override
	public void runOpMode() throws InterruptedException {

	}
}
