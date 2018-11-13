package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.team7153.HardwareByrd.DEFAULT_MOVE_SPEED;
import static org.firstinspires.ftc.team7153.HardwareByrd.HOOK;
import static org.firstinspires.ftc.team7153.HardwareByrd.LIFT_UP;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_AFT;
import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_FORE;
import static org.firstinspires.ftc.team7153.HardwareByrd.UNHOOK;


@Autonomous(name="TesterAuto")
public class TesterAuto extends AutoByrd {
	@Override
	public void runOpMode() throws InterruptedException {
		autonomousInit();
		waitForStart();
		autonomousStart();
		if (!isStopRequested()) {
			hook(UNHOOK);
			sleep(2000);
			hook(HOOK);
		}
	}
}
