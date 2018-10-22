package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_TURN_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.INPUT_TIMER;
import static org.firstinspires.ftc.team7153.HardwareByrd.PULSES_PER_REVOLUTION;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_ERROR;


public class GhettoAutoByrd extends LinearOpMode {
	HardwareByrd robot = new HardwareByrd(); //Gets robot from HardwareByrd class
	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0

	ElapsedTime runTime = new ElapsedTime();
	void autonomousInit(){


		////////////////////////////////////////////////////////////////////////////////////Hardware////////////////////////////////////////////////////////////////////////////////
		robot.init(hardwareMap);

		telemetry.addData("Gyro", " Calibrated.");
		telemetry.update();
	}

	void autonomousStart() throws InterruptedException{
		//Reset the gyroscope to account for drift

		//Reset the timer to 0
		runTime.reset();
	}

	void lift(boolean LIFT_UP){
		robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lift.setPower(1);
		if(LIFT_UP){
			robot.lift.setTargetPosition(24000);

		} else {
			robot.lift.setTargetPosition(0);
		}
		while(robot.lift.isBusy()){
			sleep(5);
			telemetry.addData("Pulses Remaining: ",24000/2-robot.lift.getCurrentPosition());
			telemetry.update();
			if(isStopRequested()){
				return;
			}
		}
		robot.lift.setPower(0);
		robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	void move(double DISTANCE, double SPEED, double DIRECTION) throws InterruptedException {
		if(isStopRequested()){
			return;
		}
        final double v1 = Math.sqrt(2) * Math.cos(DIRECTION*Math.PI/180);
        final double v2 = Math.sqrt(2) * Math.sin(DIRECTION*Math.PI/180);
        final double v3 = Math.sqrt(2) * Math.sin(DIRECTION*Math.PI/180);
        final double v4 = Math.sqrt(2) * Math.cos(DIRECTION*Math.PI/180);
        robot.frontLeft.setPower(SPEED*v1);
		robot.frontRight.setPower(SPEED*v2);
		robot.backLeft.setPower(SPEED*v3);
		robot.backRight.setPower(SPEED*v4);

		sleep((long)DISTANCE*1000);
		stopMoving();
	}

	private void resetTimer(){
		INPUT_TIMER = runTime.milliseconds();
	}

	void stopMoving() {
		//Sets the power of all motors to zero and then waits for half a second
		robot.frontLeft.setPower(0);
		robot.frontRight.setPower(0);
		robot.backLeft.setPower(0);
		robot.backRight.setPower(0);
	}

	@Override
	public void runOpMode() throws InterruptedException {

	}
}
