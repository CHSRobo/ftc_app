package org.firstinspires.ftc.team7153;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_TURN_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.INPUT_TIMER;
import static org.firstinspires.ftc.team7153.HardwareByrd.PULSES_PER_REVOLUTION;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_ERROR;


public class AutoByrd extends LinearOpMode {
	HardwareByrd robot = new HardwareByrd(); //Gets robot from HardwareByrd class
	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0

	private ElapsedTime runTime = new ElapsedTime();
	void autonomousInit(){


		////////////////////////////////////////////////////////////////////////////////////Hardware////////////////////////////////////////////////////////////////////////////////
		robot.init(hardwareMap);

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

	void autonomousStart() {
		//Reset the gyroscope to account for drift
		robot.gyro.resetZAxisIntegrator();

		//Reset the timer to 0
		runTime.reset();
	}

	void move(double DISTANCE, double SPEED, double DIRECTION) throws InterruptedException {
		if(isStopRequested()){
			return;
		}
		turn(DIRECTION,DEFAULT_TURN_SPEED);

		double deltaDistance;
		double deltaDisplacement = 0;
		double pastTime = getRuntime();

		double speedIntegral = 0;
		double angleIntegral = 0;
		double angleError = 0;

		double pastWheelAverage = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+robot.backRight.getCurrentPosition()+robot.backLeft.getCurrentPosition())/4;
		while(deltaDisplacement<DISTANCE){
			double deltaTime = getRuntime()-pastTime;
			pastTime = getRuntime();


			deltaDistance = ((robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+robot.backRight.getCurrentPosition()+robot.backLeft.getCurrentPosition())/4-pastWheelAverage);
			pastWheelAverage = deltaDistance;
			deltaDistance = 4*deltaDistance/PULSES_PER_REVOLUTION;
			double inchesPerSecond = deltaDistance/deltaTime;

			//SPEED PI
			double speedError = SPEED - inchesPerSecond;
			speedIntegral += (speedError * deltaTime);
			double speedOutput = (1/4 * speedError);// + (1 * speedIntegral);
			//

			//ANGLE PI
			angleError += Math.sin(DIRECTION)*deltaDistance;
			angleIntegral += (angleError * deltaTime);
			double angleOutput = (1/90 * angleError);// + (1 * angleIntegral);
			//

			robot.frontLeft.setPower(speedOutput+angleOutput);
			robot.frontRight.setPower(speedOutput-angleOutput);
			robot.backLeft.setPower(speedOutput+angleOutput);
			robot.backRight.setPower(speedOutput-angleOutput);

			deltaDisplacement += Math.cos(DIRECTION)*deltaDistance;
			telemetry.addData("speedError:      ", speedError);
			telemetry.addData("speedIntegral:   ", speedIntegral);
			telemetry.addData("speedOutput:     ", speedOutput);
			telemetry.addData("angleError:      ", angleError);
			telemetry.addData("angleIntegral:   ", angleIntegral);
			telemetry.addData("angleOutput:     ", angleOutput);
			telemetry.addData("inchesPerSecond: ", inchesPerSecond);
			telemetry("Move", "Main Loop");
			if(isStopRequested()){
				return;
			}
		}
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

	private void straighten() throws InterruptedException {
		//Inputs into the turn function the angle that the robot is supposed to be in
		resetTimer();

		while((robot.gyro.getHeading()<imaginaryAngle-TURN_ERROR || robot.gyro.getHeading()>imaginaryAngle+TURN_ERROR) && (imaginaryAngle-TURN_ERROR<=-1 && robot.gyro.getHeading() != 360-TURN_ERROR || imaginaryAngle-TURN_ERROR>-1) && (imaginaryAngle+TURN_ERROR>=360 && robot.gyro.getHeading()>TURN_ERROR-1 || imaginaryAngle+TURN_ERROR<360) && INPUT_TIMER+5000>runTime.milliseconds()){
			if(isStopRequested()) {
				stopMoving();
				return;
			}
			stopMoving();
			turn(imaginaryAngle,DEFAULT_TURN_SPEED);
			telemetry("Straighten", "Main Loop");
		}
	}

	private void telemetry(String FUNCTION, String PART){
		telemetry.addData("Function: ", FUNCTION + " - " + PART);
		telemetry.addData("","");
		telemetry.addData("///////DEFAULT INFORMATION","///////");
		telemetry.addData("/////MOTORS", "//////");
		telemetry.addData("FrontLeft:  ", robot.frontLeft.getPower());
		telemetry.addData("FrontRight: ", robot.frontRight.getPower());
		telemetry.addData("BackLeft:   ", robot.backLeft.getPower());
		telemetry.addData("BackRight:  ", robot.backRight.getPower());
		telemetry.addData("FrontLeft Position:  ", robot.frontLeft.getCurrentPosition());
		telemetry.addData("FrontRight Position: ", robot.frontRight.getCurrentPosition());
		telemetry.addData("BackLeft Position:   ", robot.backLeft.getCurrentPosition());
		telemetry.addData("BackRight Position:  ", robot.backRight.getCurrentPosition());
		telemetry.addData("/////SERVOS", "//////");
		telemetry.addData("Servo: ", robot.hook.getPosition());
		telemetry.addData("/////SENSOR DATA","/////");
		telemetry.addData("Gyro Heading:     ", robot.gyro.getHeading());
		telemetry.addData("Gyro IntegratedZ: ", robot.gyro.getIntegratedZValue());
		telemetry.update();
	}

	void turn(double angle, double speed) throws InterruptedException {
		if (isStopRequested()) {
			return;
		}
		//Sets the angle that the robot is supposed to be in to the angle arguement
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
				robot.frontLeft.setPower(-speed);
				robot.frontRight.setPower(speed);
				robot.backLeft.setPower(-speed);
				robot.backRight.setPower(speed);
			} else {
				robot.frontLeft.setPower(speed);
				robot.frontRight.setPower(-speed);
				robot.backLeft.setPower(speed);
				robot.backRight.setPower(-speed);
			}
			telemetry.addData("Function: ", "Turn");
			telemetry.addData("Target Angle:  ", angle);
			telemetry.addData("Current Speed: ", speed);
			telemetry("Turn" , "Main Loop");
		}
		stopMoving();
		sleep(100);
		if ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
			straighten();
		}
	}

	void hook(boolean IS_HOOK){
	    if(IS_HOOK){
	        robot.hook.setPosition(1);
        } else {
	        robot.hook.setPosition(0);
        }
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
			telemetry.addData("Pulses Remaining: ",24000-robot.lift.getCurrentPosition());
			telemetry.update();
			if(isStopRequested()){
				return;
			}
		}
		robot.lift.setPower(0);
		robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void runOpMode() throws InterruptedException {

	}
}
