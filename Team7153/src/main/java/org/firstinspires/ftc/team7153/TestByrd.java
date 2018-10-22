package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;	
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;	
import com.qualcomm.robotcore.hardware.DcMotor;

 @TeleOp(name="TestByrd")	
public class TestByrd extends OpMode{	
	private DcMotor main = null;
    @Override	
    public void init() {
       main = hardwareMap.get(DcMotor.class, "main");
       main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }	
    @Override	
    public void loop() { main.setPower(gamepad1.left_stick_y);
    telemetry.addData("Encoder", main.getCurrentPosition());
    telemetry.update();
    }
}
