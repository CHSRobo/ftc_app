package org.firstinspires.ftc.teamInventum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamInventum.HardwareInventum.RED;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.TURN_LEFT;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.TURN_RIGHT;

@Autonomous(name="RedBack")
public class RedBack extends AutoInnoMK3 {
	@Override
	public void runOpMode() throws InterruptedException {
		autonomousInit();
		waitForStart();
		autonomousStart();
		if (!isStopRequested()) {
			grab(true); // grab the block
			jewelDisplacer(RED); // displace the jewel
			move(0.3,1300); // Back off the platform
			turn(TURN_RIGHT,.3); // Turn perpandicular to the shelf
            move(-0.3,200); // Drive towards shelf
            turn(180,.3); // Turn toward shelf
            move(-0.3,500); // Go toward shelf
			grab(false); // drop the block
			move(0.3,250); // Back up a little
		}
	}
}
