package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.team7153.HardwareByrd.LIFT_DOWN;
import static org.firstinspires.ftc.team7153.HardwareByrd.UNHOOK;


@Autonomous(name="StandardAuto")
public class StandardAuto extends AutoByrd {
	@Override
	public void runOpMode() {
		autonomousInit();
		waitForStart();
		autonomousStart();
		if (!isStopRequested()) {
			lift(LIFT_DOWN);
			hook(UNHOOK);
			sleep(1000);
			moveForward(6000,.5);
			stopMoving();
		}
	}
}
