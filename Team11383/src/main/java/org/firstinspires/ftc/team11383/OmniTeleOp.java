package org.firstinspires.ftc.team11383;

/*
 * Created by Walt Morris on 9/25/17.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OmniTeleOp")

public class OmniTeleOp extends OpMode {
    
    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float r = gamepad1.right_stick_x;
        float s = gamepad2.right_stick_x;

        leftFrontMotor.setPower(x*.75-r*.75);  // Set wheels equal to left stick //
        rightFrontMotor.setPower(y*.75-r*.75);  // direction plus amount of turn, //
        rightBackMotor.setPower(-r*.75-x*.75);  //   determined by right stick.   //
        leftBackMotor.setPower(-r*.75-y*.75);
        leadScrew.setPower(s*.75); // Lead Screw extends with right, retracts with left //

        }
        if (gamepad2.a) { // hooks the hook //
            hook.setPosition(HOOK_HOOK);
            
        }
        else if (gamepad2.b) { // unhooks the hook //
            hook.setPosition(HOOK_UNHOOK);
        
        }
        else
    }
}
