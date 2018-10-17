package org.firstinspires.ftc.teamInventum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DebugMechInnoMK2")
public class DebugMechInnoMK2 extends OpMode{
	private HardwareInventum robot = new HardwareInventum();
    @Override
    public void init() {
		robot.init(hardwareMap);
		robot.color.enableLed(true);
    }

	boolean IS_BLOCK_GRAB = false;
    long INPUT_TIMER=0;

    @Override
    public void loop() {
	    double maxSpeed = 1;//Defines what fraction of speed the robot will run at

	    final double y=gamepad1.left_stick_y;
		final double x=gamepad1.left_stick_x;

		robot.leftMotor.setPower(y+x);
		robot.rightMotor.setPower(y-x);

	    /*if(gamepad2.a && System.currentTimeMillis() > INPUT_TIMER+500){
			IS_BLOCK_GRAB=!IS_BLOCK_GRAB;
			INPUT_TIMER = System.currentTimeMillis();
	    }

	    if(IS_BLOCK_GRAB){

		}*/
	    if(gamepad2.right_stick_y>.02){
			if(robot.backClamp.getCurrentPosition()<280/2.5*8) {
				robot.backClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.backClamp.setPower(0);}
			if(robot.frontClamp.getCurrentPosition()<280/2.5*7) {
				robot.frontClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.frontClamp.setPower(0);}
		} else if(gamepad2.right_stick_y<-.02){
	    	if(robot.backClamp.getCurrentPosition()>0) {
				robot.backClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.backClamp.setPower(0);}
			if(robot.frontClamp.getCurrentPosition()>0) {
				robot.frontClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.frontClamp.setPower(0);}
		} else {
			robot.backClamp.setPower(0);
			robot.frontClamp.setPower(0);
		}

		if(gamepad1.dpad_down && System.currentTimeMillis() > INPUT_TIMER+100){
			robot.rightClaw.setPosition(.01+robot.rightClaw.getPosition());
			robot.leftClaw.setPosition(-.01+robot.leftClaw.getPosition());
			INPUT_TIMER = System.currentTimeMillis();
		}
		if(gamepad1.dpad_down && System.currentTimeMillis() > INPUT_TIMER+100){
			robot.rightClaw.setPosition(-.01+robot.rightClaw.getPosition());
			robot.leftClaw.setPosition(.01+robot.leftClaw.getPosition());
			INPUT_TIMER = System.currentTimeMillis();
		}
		if(gamepad1.dpad_left){
			robot.hammer.setPosition(.01+robot.hammer.getPosition());
		} else if(gamepad1.dpad_right){
			robot.hammer.setPosition(-.01+robot.hammer.getPosition());
		}
		if(gamepad1.dpad_left && System.currentTimeMillis() > INPUT_TIMER+100){
			robot.hammer.setPosition(-.01+robot.hammer.getPosition());
			INPUT_TIMER = System.currentTimeMillis();
		}
		if(gamepad1.dpad_right && System.currentTimeMillis() > INPUT_TIMER+100){
			robot.hammer.setPosition(-.01+robot.hammer.getPosition());
			INPUT_TIMER = System.currentTimeMillis();
		}

		telemetry.addData("FrontClamp: ", robot.frontClamp.getCurrentPosition());
		telemetry.addData("BackClamp: ", robot.backClamp.getCurrentPosition());
		telemetry.addData("rightClaw:  ", robot.rightClaw.getPosition());
		telemetry.addData("leftClaw:   ", robot.leftClaw.getPosition());
		telemetry.addData("hammer:     ", robot.hammer.getPosition());
		telemetry.addData("Color Blue: ", robot.color.blue());
		telemetry.addData("Color Red:  ", robot.color.red());
	    telemetry.update();
    }
}
