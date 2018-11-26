package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_AFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_FORE;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_LEFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_RIGHT;


@Autonomous(name="TesterAuto")
public class TesterAuto extends AutoByrd {
	@Override
	public void runOpMode(){
		autonomousInit();
		waitForStart();
		autonomousStart();
		while(opModeIsActive()) {
			moveWithoutStopping(.5, MOVE_FORE);
			sleep(2000);
			moveWithoutStopping(.5, MOVE_LEFT);
			sleep(2000);
			moveWithoutStopping(.5, MOVE_AFT);
			sleep(2000);
			moveWithoutStopping(.5, MOVE_RIGHT);
			sleep(2000);
		}
		stopMoving();
	}
}
