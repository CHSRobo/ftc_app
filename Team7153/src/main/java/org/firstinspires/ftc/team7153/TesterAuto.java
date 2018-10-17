package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.team7153.HardwareByrd.BACKWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_MOVE_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.FORWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.RED;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_BACK;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_FORWARDS;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.TURN_RIGHT;
import static org.firstinspires.ftc.team7153.HardwareByrd.VUFORIA_ENABLED;

@Autonomous(name="TesterAuto")
public class TesterAuto extends AutoByrd {
	@Override
	public void runOpMode() throws InterruptedException {
		autonomousInit();
		waitForStart();
		autonomousStart();
		if (!isStopRequested()) {
			while(!isStopRequested()) {
				moveWithEncoders(10, DEFAULT_MOVE_SPEED, FORWARDS);
				turn(TURN_LEFT, DEFAULT_MOVE_SPEED);
				moveWithEncoders(10, DEFAULT_MOVE_SPEED, FORWARDS);
				turn(TURN_BACK, DEFAULT_MOVE_SPEED);
				moveWithEncoders(10, DEFAULT_MOVE_SPEED, FORWARDS);
				turn(TURN_RIGHT, DEFAULT_MOVE_SPEED);
				moveWithEncoders(10, DEFAULT_MOVE_SPEED, FORWARDS);
				turn(TURN_FORWARDS, DEFAULT_MOVE_SPEED);
			}
            /*while(runTime.milliseconds()<20000 && opModeIsActive()){
            	harvest(0,42,TURN_FORWARDS);
            	moveToCubby(TURN_RIGHT,VUFORIA_DISABLED);
			}
			stopMoving();
            intake(DUMP_INACTIVE);
            //moveToCubby2();*/
            stopMoving();
		}
	}
}
