package org.firstinspires.ftc.team7153;//package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_TURN_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.HOOK_CLOSED;
import static org.firstinspires.ftc.team7153.HardwareByrd.HOOK_OPEN;
import static org.firstinspires.ftc.team7153.HardwareByrd.INPUT_TIMER;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_FORE;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_RIGHT;
import static org.firstinspires.ftc.team7153.HardwareByrd.PULSES_PER_REVOLUTION;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_ERROR;


public class AutoByrd extends LinearOpMode {
	private HardwareByrd robot = new HardwareByrd(); //Gets robot from HardwareByrd class
	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0

	private ElapsedTime runTime = new ElapsedTime();
	void autonomousInit(){


		////////////////////////////////////////////////////////////////////////////////////Hardware////////////////////////////////////////////////////////////////////////////////
		robot.init(hardwareMap);

		//Calibrate the Gyroscope
		telemetry.addData("Gyro", " Calibrating. Do Not move!");
		telemetry.update();
		robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

		while (!isStopRequested() && robot.imu.isGyroCalibrated()) {
			sleep(50);
			idle();
		}

		telemetry.addData("Gyro", " Calibrated.");
		telemetry.update();
	}

	void autonomousStart() {
		//Reset the timer to 0
		runTime.reset();
	}

	void scan() throws InterruptedException{
		moveWithoutStopping(.5,MOVE_LEFT);
		INPUT_TIMER = getRuntime();
		while(colorDetect()==0 || INPUT_TIMER+5>getRuntime()){
			sleep(10);
		}
		if(colorDetect()==2){
			INPUT_TIMER = getRuntime()-INPUT_TIMER;
			move(6,6,MOVE_FORE);
			moveWithoutStopping(.5,MOVE_RIGHT);
			sleep((long)INPUT_TIMER);
		} else {
			moveWithoutStopping(.5,MOVE_RIGHT);
			INPUT_TIMER = getRuntime();
			while(colorDetect()==0 || INPUT_TIMER+5>getRuntime()){
				sleep(10);
			}
			if(colorDetect()==2){
				INPUT_TIMER = getRuntime()-INPUT_TIMER;
				move(6,6,MOVE_FORE);
				moveWithoutStopping(.5,MOVE_LEFT);
				sleep((long)INPUT_TIMER*1000);
			}
		}
	}

	int colorDetect(){
		float hsvValues[] = {0F, 0F, 0F};

		// values is a reference to the hsvValues array.
		final float values[] = hsvValues;

		// sometimes it helps to multiply the raw RGB values with a scale factor
		// to amplify/attentuate the measured values.
		final double SCALE_FACTOR = 255;

		Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
				(int) (robot.sensorColor.green() * SCALE_FACTOR),
				(int) (robot.sensorColor.blue() * SCALE_FACTOR),
				hsvValues);

		if(hsvValues[0]>34 && hsvValues[0]<125 && hsvValues[1]>61){
			return 1;
		} else if(hsvValues[1]<20 && hsvValues[2]>75){
			return 2;
		}else {
			return 0;
		}
	}

	void moveWithoutStopping(double SPEED, double DIRECTION){
		final double v1 = Math.sqrt(2) * Math.cos(DIRECTION*Math.PI/180);
		final double v2 = Math.sqrt(2) * Math.sin(DIRECTION*Math.PI/180);
		final double v3 = Math.sqrt(2) * Math.sin(DIRECTION*Math.PI/180);
		final double v4 = Math.sqrt(2) * Math.cos(DIRECTION*Math.PI/180);
		robot.frontLeft.setPower(SPEED*v1);
		robot.frontRight.setPower(SPEED*v2);
		robot.backLeft.setPower(SPEED*v3);
		robot.backRight.setPower(SPEED*v4);
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
		INPUT_TIMER = getRuntime();
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
		while((gyroZ()<imaginaryAngle-TURN_ERROR || gyroZ()>imaginaryAngle+TURN_ERROR) && (imaginaryAngle-TURN_ERROR<=-1 && gyroZ() != 360-TURN_ERROR || imaginaryAngle-TURN_ERROR>-1) && (imaginaryAngle+TURN_ERROR>=360 && gyroZ()>TURN_ERROR-1 || imaginaryAngle+TURN_ERROR<360) && INPUT_TIMER+5>getRuntime()){
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
		telemetry.addData("Gyro Heading:     ",gyroZ());
		telemetry.update();
	}

	private void turn(double angle, double speed) throws InterruptedException {
		if (isStopRequested()) {
			return;
		}
		//Sets the angle that the robot is supposed to be in to the angle argument
		imaginaryAngle = angle;

		float pastAngle = gyroZ(), currentAngle = gyroZ(), deltaAngle = 0;
		double pastTime = getRuntime(), currentTime = getRuntime(), deltaTime = 0;
		double anglePerSecond;
		double speedIntegral = 0;
		//While the angel is > the gyroscope+TURN_ERROR or < the gyroscope-TURN_ERROR
		resetTimer();
		while ((gyroZ() < angle - TURN_ERROR || gyroZ() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && gyroZ() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && gyroZ() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5 > getRuntime()) {
			if (isStopRequested()) {
				stopMoving();
				return;
			}

            currentAngle = gyroZ();
            currentTime = getRuntime();
            deltaAngle = Math.abs(currentAngle-pastAngle);
            deltaTime = currentTime-pastTime;
            anglePerSecond = deltaAngle/deltaTime;
            pastAngle=currentAngle;
            pastTime=currentTime;
            double speedError = speed - anglePerSecond;
            speedIntegral += (speedError * deltaTime);
            double speedOutput = (1/8 * speedError) + (.001 * speedIntegral);

			//Checks to see if turning left or right
			if ((angle > gyroZ() && angle < gyroZ() + 181) || (angle < gyroZ() - 180)) {
				robot.frontLeft.setPower(-speedOutput);
				robot.frontRight.setPower(speedOutput);
				robot.backLeft.setPower(-speedOutput);
				robot.backRight.setPower(speedOutput);
			} else {
				robot.frontLeft.setPower(speedOutput);
				robot.frontRight.setPower(-speedOutput);
				robot.backLeft.setPower(speedOutput);
				robot.backRight.setPower(-speedOutput);
			}

			telemetry.addData("Function: ", "Turn");
			telemetry.addData("Target Angle:  ", angle);
			telemetry.addData("Current Speed: ", speed);
			telemetry("Turn" , "Main Loop");
		}
		stopMoving();
		sleep(100);
		if ((gyroZ() < angle - TURN_ERROR || gyroZ() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && gyroZ() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && gyroZ() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5 > getRuntime()) {
			straighten();
		}
	}

	void hook(boolean IS_HOOK){
	    if(IS_HOOK){
	        robot.hook.setPosition(HOOK_CLOSED);
        } else {
	        robot.hook.setPosition(HOOK_OPEN);
        }
    }

    void moveForward (long time, double power){
		robot.frontLeft.setPower(power);
		robot.frontRight.setPower(power);
		robot.backLeft.setPower(power);
		robot.backRight.setPower(power);
		sleep(time);
	}

	void lift(boolean LIFT_UP){
		robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lift.setPower(1);
		if(LIFT_UP){
			robot.lift.setTargetPosition(29000);

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

	private float gyroZ () {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.firstAngle + 180;
    }

	@Override
	public void runOpMode() throws InterruptedException {

	}
}
