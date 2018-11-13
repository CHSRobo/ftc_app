package org.firstinspires.ftc.team11383;

/*
 * Created by Walt Morris on 9/25/17.
 * Made better by Will Archer on 10/20/18
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OmniTeleOp")

public class OmniTeleOp extends OpMode{
    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }


    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float r = gamepad1.right_stick_x;
        float s = gamepad2.right_stick_x;


        robot.leftFrontMotor.setPower(x * .75 + r * .75);  // Set wheels equal to left stick //
        robot.rightFrontMotor.setPower(- y * .75 + r * .75);  // direction plus amount of turn, //
        robot.rightBackMotor.setPower( r * .75 - x * .75);  //   determined by right stick.   //
        robot.leftBackMotor.setPower( r * .75 + y * .75);
        robot.leadScrew.setPower(s); // Lead Screw extends with right, retracts with left //

        if (gamepad2.a) { // hooks the hook //
            robot.hook.setPosition(robot.HOOK_HOOK);

        } else if (gamepad2.b) { // unhooks the hook //
            robot.hook.setPosition(robot.HOOK_UNHOOK);
        
        } else if (gamepad2.dpad_up) { // pusher up //
            robot.push.setPosition(robot.PUSH_UP);
            
        } else if (gamepad2.dpad_right) { // pusher mid //
            robot.push.setPosition(robot.PUSH_DOWN);

        } else if (gamepad2.dpad_left) { // pusher mid //
            robot.push.setPosition(robot.PUSH_MIDUP);

        } else if (gamepad2.dpad_down) { // pusher down //
            robot.push.setPosition(robot.PUSH_MIDDOWN);
        }
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
