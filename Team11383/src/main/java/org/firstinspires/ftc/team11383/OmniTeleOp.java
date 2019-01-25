// OmniTeleOp.java
// Antiquated Driver-controlled OpMode for Omniwheels

package org.firstinspires.ftc.team11383;

/*
 * Obsolete as of 11/26/2018
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


        if (gamepad2.dpad_up) { // pusher angle a - highest //
            robot.push.setPosition(robot.PUSH_A);
            
        } else if (gamepad2.dpad_right) { // pusher angle d - lowest //
            robot.push.setPosition(robot.PUSH_D);

        } else if (gamepad2.dpad_left) { // pusher angle b - high //
            robot.push.setPosition(robot.PUSH_B);

        } else if (gamepad2.dpad_down) { // pusher angle c - low //
            robot.push.setPosition(robot.PUSH_C);
        }
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
