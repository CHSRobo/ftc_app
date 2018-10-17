package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.team7153.HardwareByrd.BACKWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.BLOCK_NO_PUSH;
import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_MOVE_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_TURN_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.DUMP_DOWN;
import static org.firstinspires.ftc.team7153.HardwareByrd.DUMP_EXPEL;
import static org.firstinspires.ftc.team7153.HardwareByrd.DUMP_INACTIVE;
import static org.firstinspires.ftc.team7153.HardwareByrd.DUMP_INTAKE;
import static org.firstinspires.ftc.team7153.HardwareByrd.DUMP_UP;
import static org.firstinspires.ftc.team7153.HardwareByrd.FORWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.GYRO_MOVE;
import static org.firstinspires.ftc.team7153.HardwareByrd.HAMMER_CENTER;
import static org.firstinspires.ftc.team7153.HardwareByrd.HAMMER_DOWN;
import static org.firstinspires.ftc.team7153.HardwareByrd.HAMMER_LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.HAMMER_RIGHT;
import static org.firstinspires.ftc.team7153.HardwareByrd.HAMMER_UP;
import static org.firstinspires.ftc.team7153.HardwareByrd.INPUT_TIMER;
import static org.firstinspires.ftc.team7153.HardwareByrd.LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_BACKWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.OFFSET;
import static org.firstinspires.ftc.team7153.HardwareByrd.RED;
import static org.firstinspires.ftc.team7153.HardwareByrd.RIGHT;
import static org.firstinspires.ftc.team7153.HardwareByrd.SLOT_1;
import static org.firstinspires.ftc.team7153.HardwareByrd.SLOT_2;
import static org.firstinspires.ftc.team7153.HardwareByrd.SLOT_3;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_BACK;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_ERROR;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_FORWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_RIGHT;


public class AutoByrd extends LinearOpMode {
	HardwareByrd robot = new HardwareByrd(); //Gets robot from HardwareByrd class
	private double imaginaryAngle=0;         //Sets the robot's initial angle to 0

	ElapsedTime runTime = new ElapsedTime();

	private VuforiaTrackable relicTemplate;
	private RelicRecoveryVuMark relicVuMark = RelicRecoveryVuMark.UNKNOWN;
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

		////////////////////////////////////////////////////////////////////////////////////Vuforia/////////////////////////////////////////////////////////////////////////////////

		//Get the view for the camera monitor
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

		//Use Vindicem license key
		parameters.vuforiaLicenseKey = "ARK0G5D/////AAAAGTNyS/9bI0eKk0BiZlza4w8qOLSfAS/JLHbvWMY95VY7PgFNgH178LKZTQVDke1Eu9JzX/o9QWeyU5ottyCSuPaRr98YId9QUZtfX918roLvNx3n5bXekGlcKSoxgw+UcH3HN+c8V57B3fFhNMt0uyKEWNAXYmAx1OkvoFUSSurH82uzsGg+aBZ3nlVfj043RPXSDyiJO7uDZmwVH14LPjdhP92Qj6byGdICOqc5dxKG1rVFdNgAWJjYVWbz53K1qNWyO9fYgE0lIjwgNopM2GCFVR2ycS0JHx5UW3Bk2m47kDoFCFJP+A8fWxfLyrtgH02JOzNyHb0VoKv4ZDan5Czl7Wcs+ItJBby3qyEmPRkf";

		//Assign which camera is to be used
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		//Load the VuMarks
		VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
		relicTemplate = relicTrackables.get(0);

		relicTemplate.setName("relicVuMarkTemplate");

		telemetry.addData("Vuforia", " Activating. Do Not Start!");
		telemetry.update();

		relicTrackables.activate();
		telemetry.addData("Vuforia", " Activated. Start");
		telemetry.update();
	}

	void autonomousStart() throws InterruptedException{
		//Reset the gyroscope to account for drift
		robot.gyro.resetZAxisIntegrator();

		//Reset the timer to 0
		runTime.reset();

		robot.blockPusher.setPosition(BLOCK_NO_PUSH);

		statusCheck();
	}

	private void dump(boolean mode) throws InterruptedException{
		if(mode){
			robot.dump.setPower(.3);
			robot.dump.setTargetPosition(DUMP_UP);
		} else{
			robot.dump.setTargetPosition(DUMP_DOWN);
			robot.dump.setPower(.1);
		}
	}
	
	void hammer(boolean colorRemaining) throws InterruptedException {
		if(isStopRequested()){
			return;
		}
		//Brings the hammer down and turns on the LED
		robot.color.enableLed(true);
		robot.hammerY.setPosition(HAMMER_DOWN);
		sleep(500);
		if(isStopRequested()){
			return;
		}
		/*If the color red is greater than the color blue
		*then, if the argument is red it will putt the blue ball off (Left)
		*otherwise it will putt the red ball off (Right)
		*/
		for(double increment=0; robot.color.red()==robot.color.blue() && increment<.25; increment+=.01){
			robot.hammerX.setPosition(HAMMER_CENTER+increment/10);
			telemetry.addData("Hammer Position: ", "%3.2f",HAMMER_CENTER+increment);
			telemetry();
			sleep(10);
		}
		if(robot.color.red()>=robot.color.blue()){
			telemetry.addData("Found Color: ", "red");
			telemetry.update();
			if(colorRemaining==RED){putt(RIGHT);} else {putt(LEFT);}
		} else if (robot.color.red()<=robot.color.blue()){
			/*If the color red is less than the color blue
			*then, if the argument is red it will putt the blue ball off (Right)
			*otherwise it will putt the red ball off (Left)
			*/
			telemetry.addData("Found Color: ", "blue");
			telemetry.update();
			if(colorRemaining==RED){putt(LEFT);} else {putt(RIGHT);}
		} else {
			//If no color is found then nothing will occur
			telemetry.addData("Found Color: ", "Nothing");
			telemetry.update();
		}
		statusCheck();
		sleep(500);
		statusCheck();
		//Reset the hammer position to up and turn off the LED
		robot.hammerY.setPosition(HAMMER_UP);
		sleep(250);
		robot.hammerX.setPosition(HAMMER_CENTER);
		robot.color.enableLed(false);
	}

	void harvest(double X_AXIS, double Y_AXIS,double END_DIRECTION) throws InterruptedException{
		GYRO_MOVE=true;
		turn(TURN_RIGHT,.5);
		//Open up the clamp and activate the intake in preparation of blocks.
		intake(DUMP_INTAKE);
		//Move away from the wall for Y_AXIS inches.
		moveWithEncoders(Y_AXIS,.6,FORWARDS);
		/* X_AXIS is based off of the bot's forward direction.
		* Forwards is positive.
		* Backwards is negative.
		* So if X_AXIS is a positive number then it will turn forwards functions.
		* If the X_AXIS is a negative number then it will turn backwards.
		* If the robot doesn't have to move in the X direction then it will assume it has 
		* reached its destination and attempt to pick up a block.
		* After looping the first time it will then correct for the bot's offset.
		*
		*/
		if(X_AXIS>0){
			turn(TURN_FORWARDS,DEFAULT_TURN_SPEED);
			moveWithEncoders(X_AXIS,.4,FORWARDS);
			moveWithEncoders(X_AXIS,.4,BACKWARDS);
		} else if (X_AXIS<0) {
			turn(TURN_BACK,DEFAULT_TURN_SPEED);
			moveWithEncoders(-X_AXIS,.4,FORWARDS);
			moveWithEncoders(-X_AXIS,.4,BACKWARDS);
		}
		intake(DUMP_INACTIVE);
		//Add the offset incurred by placing the cryptoblock
		/*
		* Return using the Y_AXIS now going back towards the cryptobox
		*/
		turn(TURN_RIGHT,DEFAULT_TURN_SPEED);
		moveWithEncoders(Y_AXIS,.6,BACKWARDS);
		turn(END_DIRECTION,DEFAULT_TURN_SPEED);
		if(OFFSET>0) {
			moveWithEncoders(OFFSET, DEFAULT_MOVE_SPEED, FORWARDS);
		} else {
			moveWithEncoders(-OFFSET,DEFAULT_MOVE_SPEED, BACKWARDS);
		}
		OFFSET = 0;
	}

	void intake(double mode) {
		robot.intakeFrontLeft.setPower(mode);
		robot.intakeFrontRight.setPower(mode);
		robot.intakeBackLeft.setPower(mode);
		robot.intakeBackRight.setPower(mode);
	}


	void moveWithEncoders(double distance, double power, boolean direction) throws InterruptedException {
		if(isStopRequested()){
			return;
		}
		straighten();
		if(!direction){distance*=-1;}

		robot.frontLeft.setTargetPosition((int)(robot.frontLeft.getCurrentPosition()+(distance/4)*280));
		robot.frontRight.setTargetPosition((int)(robot.frontRight.getCurrentPosition()+(distance/4)*280));
		robot.backLeft.setTargetPosition((int)(robot.backLeft.getCurrentPosition()+(distance/4)*280));
		robot.backRight.setTargetPosition((int)(robot.backRight.getCurrentPosition()+(distance/4)*280));

		robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		resetTimer();

		while(robot.frontRight.getTargetPosition()!=robot.frontRight.getCurrentPosition() && INPUT_TIMER+5000>runTime.milliseconds()){
			if(isStopRequested()){
				stopMoving();
				return;
			}
			robot.frontLeft.setPower(power);
			robot.frontRight.setPower(power);
			robot.backLeft.setPower(power);
			robot.backRight.setPower(power);
			telemetry();
			sleep(10);
		}
		sleep(100);
		robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	void moveToCubby(double turnDirection, boolean mode) throws InterruptedException {
		if (isStopRequested()) {
			return;
		}
		straighten();
		//While the hammer is not detecting any color or the time-out hasn't occurred the robot will move towards a cubby slot
		if(!SLOT_2 && !mode){
			SLOT_2=true;
			OFFSET = 0;
		} else if ((relicVuMark == RelicRecoveryVuMark.LEFT && mode) || (!SLOT_1 && !mode)) {
			SLOT_1 = true;
			OFFSET = -7.63;
			moveWithEncoders(7.63,DEFAULT_MOVE_SPEED,BACKWARDS);
		} else if ((relicVuMark == RelicRecoveryVuMark.RIGHT && mode) || (!SLOT_3 && !mode)) {
			SLOT_3 = true;
			OFFSET = 7.63;
			moveWithEncoders(7.63,DEFAULT_MOVE_SPEED,FORWARDS);
		} else if (mode) {
			SLOT_2 = true;
		}
		turn(turnDirection,DEFAULT_TURN_SPEED);
		straighten();
		moveWithEncoders(8,DEFAULT_MOVE_SPEED,BACKWARDS);
		straighten();
		dump(true);
		sleep(1000);
		/*sleep(1000);
		moveWithoutStopping(turnDirection,.3);
		sleep(500);
		moveWithoutStopping(turnDirection+90,.3);
		sleep(500);
		moveWithoutStopping(turnDirection-90,.3);
		sleep(1000);
		moveWithoutStopping(turnDirection+90,.15);
		sleep(500);*/
		dump(false);
		moveWithEncoders(10,DEFAULT_MOVE_SPEED,FORWARDS);
		straighten();
	}
	void moveToCubby2() throws InterruptedException{
		turn(TURN_LEFT,.3);
		for(int x=0; x<2;x++) {
			double distance = robot.cubbyRange.cmUltrasonic();
			moveWithoutStopping(MOVE_BACKWARDS, 1);
			while (robot.cubbyRange.cmUltrasonic() > distance - 5) {
				telemetry();
				sleep(10);
			}
			stopMoving();
			sleep(5000);
		}
		intake(DUMP_EXPEL);
		sleep(1000);
		intake(DUMP_INACTIVE);
	}

	private void moveWithoutStopping(double angle, double power) throws InterruptedException {
		//See the move function. Just doesn't have the stopMoving() function built in.
		if(isStopRequested()){
			return;
		}

		straighten();

		double radGyro = (robot.gyro.getIntegratedZValue() * Math.PI) / 180;
		double robotAngle = angle * (Math.PI / 180) - Math.PI / 4 - radGyro;

		if (robotAngle > Math.PI) {
			robotAngle = Math.PI * -2 + robotAngle;
		} else if (robotAngle < -Math.PI) {
			robotAngle = Math.PI * 2 + robotAngle;
		}

		final double v1 = power * Math.cos(robotAngle);
		final double v2 = power * Math.sin(robotAngle);
		final double v3 = power * Math.sin(robotAngle);
		final double v4 = power * Math.cos(robotAngle);

		robot.frontLeft.setPower(v1);
		robot.frontRight.setPower(v2);
		robot.backLeft.setPower(v3);
		robot.backRight.setPower(v4);

		telemetry.addData("Function: ", "MoveWithoutStopping");
		telemetry.addData("Target Angle:  ", angle);
		telemetry.addData("Current Speed: ", power);
		telemetry();
	}
	
	private void putt(boolean direction){
		if(isStopRequested()){
			return;
		}
		//If the argument says right or left then the hammer will putt to the respective positions
		if(direction == RIGHT){
			telemetry.addData("Hammer Position: ", "Right");
			telemetry();
			robot.hammerX.setPosition(HAMMER_RIGHT);
		} else {
			telemetry.addData("Hammer Position: ", "Left");
			telemetry();
			robot.hammerX.setPosition(HAMMER_LEFT);
		}
	}

	private void resetTimer(){
		INPUT_TIMER = runTime.milliseconds();
	}

	private void statusCheck() throws InterruptedException{
		if(isStopRequested()){
			stopMoving();
			intake(DUMP_INACTIVE);
			return;
		}

		if(RelicRecoveryVuMark.from(relicTemplate) != RelicRecoveryVuMark.UNKNOWN){
			relicVuMark = RelicRecoveryVuMark.from(relicTemplate);
		}
	}

	void stopMoving() throws InterruptedException {
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
			turn(imaginaryAngle,.24);
			telemetry();
		}
	}

	private void telemetry(){
		telemetry.addData("/////VUFORIA", "/////");
		telemetry.addData("VuMark", "%s visible", relicVuMark);
		telemetry.addData("/////SENSORS", "/////");
		telemetry.addData("Color Red:     ", robot.color.red());
		telemetry.addData("Color Blue:    ", robot.color.blue());
		telemetry.addData("Current Angle: ", robot.gyro.getIntegratedZValue());
		telemetry.addData("/////ENCODERS", "////");
		telemetry.addData("Move Target Position:  ", robot.frontLeft.getTargetPosition());
		telemetry.addData("Move Current Position: ", robot.frontLeft.getCurrentPosition());
		telemetry.addData("Lift Target Position:  ", robot.lift.getTargetPosition());
		telemetry.addData("Lift Current Position: ", robot.lift.getCurrentPosition());
		telemetry.addData("Dump Target Position:  ", robot.dump.getTargetPosition());
		telemetry.addData("Dump Current Position: ", robot.dump.getCurrentPosition());
		telemetry.addData("/////MOTORS", "//////");
		telemetry.addData("FrontLeft:  ", robot.frontLeft.getPower());
		telemetry.addData("FrontRight: ", robot.frontRight.getPower());
		telemetry.addData("BackLeft:   ", robot.backLeft.getPower());
		telemetry.addData("BackRight:  ", robot.backRight.getPower());
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
			/*if(INPUT_TIMER_2 + 5 < runTime.milliseconds() && motorSpeed<speed) {
				motorSpeed+=speed/100;
				INPUT_TIMER_2 = runTime.milliseconds();
			}*/
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
			telemetry();
		}
		stopMoving();
		sleep(100);
		if ((robot.gyro.getHeading() < angle - TURN_ERROR || robot.gyro.getHeading() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && robot.gyro.getHeading() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && robot.gyro.getHeading() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5000 > runTime.milliseconds()) {
			straighten();
		}
	}

	@Override
	public void runOpMode() throws InterruptedException {

	}
}
