package org.firstinspires.ftc.team7153;//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import static org.firstinspires.ftc.teamcode.HardwareByrd.DEFAULT_TURN_SPEED;
//import static org.firstinspires.ftc.teamcode.HardwareByrd.INPUT_TIMER;
//import static org.firstinspires.ftc.teamcode.HardwareByrd.PULSES_PER_REVOLUTION;
//import static org.firstinspires.ftc.teamcode.HardwareByrd.TURN_ERROR;
//
//
//public class AutoByrd extends LinearOpMode {
//	HardwareByrd robot = new HardwareByrd(); //Gets robot from HardwareByrd class
//	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0
//
//	ElapsedTime runTime = new ElapsedTime();
//	void autonomousInit(){
//
//
//		////////////////////////////////////////////////////////////////////////////////////Hardware////////////////////////////////////////////////////////////////////////////////
//		robot.init(hardwareMap);
//
//		//Calibrate the Gyroscope
//		telemetry.addData("Gyro", " Calibrating. Do Not move!");
//		telemetry.update();
//		robot.gyro.calibrate();
//
//		while (!isStopRequested() && robot.gyro.isCalibrating()) {
//			sleep(50);
//			idle();
//		}
//
//		telemetry.addData("Gyro", " Calibrated.");
//		telemetry.update();
//	}
//
//	void autonomousStart() throws InterruptedException{
//		//Reset the gyroscope to account for drift
//		robot.gyro.resetZAxisIntegrator();
//
//		//Reset the timer to 0
//		runTime.reset();
//	}
//
//	void lift(boolean LIFT_UP){
//		if(LIFT_UP){
//			robot.lift.setPower(1);
//			while(robot.lift.getCurrentPosition()<144){
//				sleep(5);
//			}
//		} else {
//			robot.lift.setPower(-1);
//			while(robot.lift.getCurrentPosition()>0){
//				sleep(5);
//			}
//		}
//	}
//
//	void move(double DISTANCE, double SPEED, double DIRECTION) throws InterruptedException {
//		if(isStopRequested()){
//			return;
//		}
//		turn(DIRECTION,DEFAULT_TURN_SPEED);
//
//		double deltaDistance = 0;
//		double deltaDisplacement = 0;
//		double pastTime = getRuntime();
//
//		double speedIntegral = 0;
//		double angleIntegral = 0;
//		double angleError = 0;
//
//		double pastWheelAverage = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+robot.backRight.getCurrentPosition()+robot.backLeft.getCurrentPosition())/4;
//		while(deltaDisplacement<DISTANCE){
//			double deltaTime = getRuntime()-pastTime;
//			pastTime = getRuntime();
//
//
//			deltaDistance = ((robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+robot.backRight.getCurrentPosition()+robot.backLeft.getCurrentPosition())/4-pastWheelAverage);
//			pastWheelAverage = deltaDistance;
//			deltaDistance = 4*deltaDistance/PULSES_PER_REVOLUTION;
//			double inchesPerSecond = deltaDistance/deltaTime;
//
//			//SPEED PI
//			double speedError = SPEED - inchesPerSecond;
//			speedIntegral += (speedError * deltaTime);
//			double speedOutput = (1 * speedError) + (1 * speedIntegral);
//			//
//
//			//ANGLE PI
//			angleError += Math.sin(DIRECTION)*deltaDistance;
//			angleIntegral += (angleError * deltaTime);
//			double angleOutput = (1 * angleError) + (1 * angleIntegral);
//			//
//
//			robot.frontLeft.setPower(speedOutput+angleOutput);
//			robot.frontRight.setPower(speedOutput-angleOutput);
//			robot.backLeft.setPower(speedOutput+angleOutput);
//			robot.backRight.setPower(speedOutput-angleOutput);
//
//			deltaDisplacement += Math.cos(DIRECTION)*deltaDistance;
//			stopMoving();
//		}
//	}
//
//	private void resetTimer(){
//		INPUT_TIMER = runTime.milliseconds();
//	}
//
//	void stopMoving() {
//		//Sets the power of all motors to zero and then waits for half a second
//		robot.frontLeft.setPower(0);
//		robot.frontRight.setPower(0);
//		robot.backLeft.setPower(0);
//		robot.backRight.setPower(0);
//	}
//
//	private void straighten() throws InterruptedException {
//		//Inputs into the turn function the angle that the robot is supposed to be in
//		resetTimer();
//
//		while((robot.gyro.getHeading()<imaginaryAngle-TURN_ERROR || robot.gyro.getHeading()>imaginaryAngle+TURN_ERROR) && (imaginaryAngle-TURN_ERROR<=-1 && robot.gyro.getHeading() != 360-TURN_ERROR || imaginaryAngle-TURN_ERROR>-1) && (imaginaryAngle+TURN_ERROR>=360 && robot.gyro.getHeading()>TURN_ERROR-1 || imaginaryAngle+TURN_ERROR<360) && INPUT_TIMER+5000>runTime.milliseconds()){
//			if(isStopRequested()) {
//				stopMoving();
//				return;
//			}
//			stopMoving();
//			turn(imaginaryAngle,DEFAULT_TURN_SPEED);
//			telemetry();
//		}
//	}
//
//	private void telemetry(){
//		telemetry.addData("/////MOTORS", "//////");
//		telemetry.addData("FrontLeft:  ", robot.frontLeft.getPower());
//		telemetry.addData("FrontRight: ", robot.frontRight.getPower());
//		telemetry.addData("BackLeft:   ", robot.backLeft.getPower());
//		telemetry.addData("BackRight:  ", robot.backRight.getPower());
//		telemetry.update();
//	}
//
//	void turn(double angle, double speed) throws InterruptedException {
//		if (isStopRequested()) {
//			return;
//		}
//		//Sets the angle that the robot is supposed to be in to the angle arguement
//		imaginaryAngle = angle;
//		//While the angel is > the gyroscope+TURN_ERROR or < the gyroscope-TURN_ERROR
//		resetTimer();
//		while ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
//			if (isStopRequested()) {
//				stopMoving();
//				return;
//			}
//
//			//Checks to see if turning left or right
//			if ((angle > robot.gyro.getHeading() && angle < robot.gyro.getHeading() + 181) || (angle < robot.gyro.getHeading() - 180)) {
//				robot.frontLeft.setPower(-speed);
//				robot.frontRight.setPower(speed);
//				robot.backLeft.setPower(-speed);
//				robot.backRight.setPower(speed);
//			} else {
//				robot.frontLeft.setPower(speed);
//				robot.frontRight.setPower(-speed);
//				robot.backLeft.setPower(speed);
//				robot.backRight.setPower(-speed);
//			}
//			telemetry.addData("Function: ", "Turn");
//			telemetry.addData("Target Angle:  ", angle);
//			telemetry.addData("Current Speed: ", speed);
//			telemetry();
//		}
//		stopMoving();
//		sleep(100);
//		if ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
//			straighten();
//		}
//	}
//
//	@Override
//	public void runOpMode() throws InterruptedException {
//
//	}
//}
