package org.firstinspires.ftc.teamInventum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamInventum.HardwareInventum.LEFT_CLAMP_CLOSE;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.LEFT_CLAMP_OPEN;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.RIGHT_CLAMP_CLOSE;
import static org.firstinspires.ftc.teamInventum.HardwareInventum.RIGHT_CLAMP_OPEN;

@TeleOp(name="TankInnoMK2")
public class TankInnoMK2 extends OpMode{
	private HardwareInventum robot = new HardwareInventum();
    @Override
    public void init() {
		robot.init(hardwareMap);
		robot.color.enableLed(false);
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

	    if(gamepad2.x && System.currentTimeMillis() > INPUT_TIMER+500){
			IS_BLOCK_GRAB=!IS_BLOCK_GRAB;
			INPUT_TIMER = System.currentTimeMillis();
	    }

	    if(IS_BLOCK_GRAB){
			robot.leftClaw.setPosition(LEFT_CLAMP_CLOSE);
			robot.rightClaw.setPosition(RIGHT_CLAMP_CLOSE);
		} else {
			robot.leftClaw.setPosition(LEFT_CLAMP_OPEN);
			robot.rightClaw.setPosition(RIGHT_CLAMP_OPEN);
		}

		if(-gamepad2.right_stick_y<-.02){
			if(robot.backClamp.getCurrentPosition()<(16/1.25)*280) {//(inches to travel/circumference)*PPR
				robot.backClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.backClamp.setPower(0);}
			if(robot.frontClamp.getCurrentPosition()<(14/1.25)*280) {//(inches to travel/circumference)*PPR
				robot.frontClamp.setPower(gamepad2.right_stick_y * 1);
			} else {robot.frontClamp.setPower(0);}
		} else if(-gamepad2.right_stick_y>.02){
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
