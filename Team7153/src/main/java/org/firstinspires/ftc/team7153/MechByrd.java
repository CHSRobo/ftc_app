package org.firstinspires.ftc.team7153;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.team7153.HardwareByrd.HOOK_CLOSED;
import static org.firstinspires.ftc.team7153.HardwareByrd.HOOK_OPEN;

@TeleOp(name="MechByrd")
public class MechByrd extends OpMode{	
	private HardwareByrd robot = new HardwareByrd();
    Orientation angles;
    private double inputTimer = 0;
    private boolean isHooked = false;
    @Override
    public void init() {
        robot.init(hardwareMap);
    }	
    @Override	
    public void loop() {
        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
		double maxSpeed = .7;//Defines what fraction of speed the robot will run at
		//double radGyro = ((angles.firstAngle+180) * Math.PI) / 180;
		double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
		double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;;// - radGyro;
		if(gamepad1.right_bumper){robotAngle=Math.PI*7/4; r=1;}
		else if (gamepad1.left_bumper){robotAngle=Math.PI*3/4; r=1;}
		double rightX = 1.667*gamepad1.right_stick_x;
		final double v1 = r * Math.sqrt(2) * Math.cos(robotAngle) + rightX;
		final double v2 = r * Math.sqrt(2) * Math.sin(robotAngle) - rightX;
		final double v3 = r * Math.sqrt(2) * Math.sin(robotAngle) + rightX;
		final double v4 = r * Math.sqrt(2) * Math.cos(robotAngle) - rightX;

		robot.frontLeft.setPower(v1*maxSpeed);
		robot.frontRight.setPower(v2*maxSpeed);
		robot.backLeft.setPower(v3*maxSpeed);
		robot.backRight.setPower(v4*maxSpeed);

    	if(Math.abs(gamepad2.left_stick_y)>.05){// && (-gamepad2.left_stick_y>0 || robot.lift.getCurrentPosition()>0)){
    	    robot.lift.setPower(-gamepad2.left_stick_y);
        } else {
    	    robot.lift.setPower(0);
        }

        if(inputTimer+.2<getRuntime()) {
            if (gamepad2.a) {
                isHooked = !isHooked;
                if(isHooked){robot.hook.setPosition(HOOK_CLOSED);}
                else{robot.hook.setPosition(HOOK_OPEN);}
                inputTimer = getRuntime();
            }
        }

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("////Gamepad", " Info////");
        telemetry.addData("//Gamepad", " 1//");
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("//Gamepad", " 2//");
        telemetry.addData("Left Stick Y: ", gamepad2.left_stick_y);
        telemetry.addData("////Sensors", "////");
        telemetry.addData("Gyroscope First:  ", angles.firstAngle );
        telemetry.addData("Gyroscope Second: ", angles.secondAngle );
        telemetry.addData("Gyroscope Third:  ", angles.thirdAngle );
        telemetry.addData("Color Red:   ", robot.sensorColor.red());
        telemetry.addData("Color Green: ", robot.sensorColor.green());
        telemetry.addData("Color Blue:  ", robot.sensorColor.blue());
        telemetry.addData("Color Alpha: ", robot.sensorColor.alpha());
        telemetry.addData("Color H: ", hsvValues[0]);
        telemetry.addData("Color S:  ", hsvValues[1]);
        telemetry.addData("Color V: ", hsvValues[2]);
        telemetry.addData("////Motor", " Info////");
        telemetry.addData("FrontLeft Power: ", robot.frontLeft.getPower());
        telemetry.addData("FrontRight Power: ", robot.frontRight.getPower());
        telemetry.addData("BackLeft Power: ", robot.backLeft.getPower());
        telemetry.addData("BackRight Power: ", robot.backRight.getPower());
    	telemetry.addData("Lift Position: ", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Power: ", robot.lift.getPower());
        telemetry.addData("////Servo", " Info////");
        telemetry.addData("Hook position: ", robot.hook.getPosition());
        telemetry.addData("Is Hooked:     ", isHooked);
        telemetry.addData("RunTime: ", getRuntime());
        telemetry.addData("InputTimer: ", inputTimer);
        telemetry.update();
    }	
}