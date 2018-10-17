package org.firstinspires.ftc.teamInventum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamInventum.HardwareInventum.BLUE;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.RED;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.TURN_FORWARDS;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.TURN_LEFT;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.TURN_RIGHT;

@Autonomous(name="BlueBack")
public class BlueBack extends AutoInnoMK3 {
	@Override
	public void runOpMode() throws InterruptedException {
		autonomousInit();
		waitForStart();
		autonomousStart();
		if (!isStopRequested()) {
			grab(true); // grab the block
			sleep(200);
			robot.frontClamp.setPower(1); sleep(400); robot.frontClamp.setPower(0); // raise the clamp
			jewelDisplacer(BLUE); // displace the jewel
			move(-0.3,850 ); // Drive off the platform
			turn(270+45,.3); // Turn 45 degrees to the shelf
            move(-0.3,700); // Drive towards shelf
           // turn(0,.3); // Turn toward shelf
           // move(-0.3,500); // Go toward shelf
			grab(false); // drop the block
			move(0.3,250); // Back up a little
		}
	}
}
