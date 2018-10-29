//package org.firstinspires.ftc.team7153;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import static org.firstinspires.ftc.team7153.HardwareByrd.LIFT_UP;
//import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_AFT;
//import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_FORE;
//import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_LEFT;
//import static org.firstinspires.ftc.team7153.HardwareByrd.MOVE_RIGHT;
//import static org.firstinspires.ftc.team7153.HardwareByrd.UNHOOK;
//
//
//@Autonomous(name="StandardAuto")
//public class StandardAuto extends AutoByrd {
//	@Override
//	public void runOpMode() throws InterruptedException {
//		autonomousInit();
//		waitForStart();
//		autonomousStart();
//		if (!isStopRequested()) {
//			lift(LIFT_UP);
//			hook(UNHOOK);
//			move(6,.5, MOVE_AFT);
//			stopMoving();
//		}
//	}
//}
