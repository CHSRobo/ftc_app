/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.team11750;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Omni Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Bellatorum: TeleopIntuitive", group="Bellatorum")
public class TeleopIntuitive extends OpMode{

    /* Declare OpMode members. */
    HardwareBellatorum robot       = new HardwareBellatorum(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clampOffset  = 0.0 ;                  // Servo mid position
    final double    CLAMP_SPEED  = 0.02 ;                 // sets rate to move servo
    private boolean drill=false; // Initially disable drill mode
    ModernRoboticsI2cGyro gyro; // Gyroscope Sensor //

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // make sure the gyro is calibrated before continuing
        // Calibrate the gyro
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Read the x,y displacement from the left stick
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float r = gamepad1.right_stick_x; // Read the rotation from the right stick

        // Robot Translate/Slide mode
        if(gamepad1.x) {
            drill = !drill; // Toggle drill mode
            gyro.resetZAxisIntegrator(); // Re-orient the bot
        }
        if(drill) {
            double z=Math.toRadians(gyro.getIntegratedZValue()); // Get anchor heading
            double angle=0;
            if (x!=0 || y!=0) angle=Math.atan(-y/x); // Calculate the new desired direction

            double power=Math.sqrt(x*x+y*y); // Calculate the desired power
            // Set the wheels to the send the bot towards the desired direction less the z anchor + rotation
            robot.leftFrontMotor.setPower(power*Math.cos(angle-z)+r);
            robot.rightFrontMotor.setPower(-power*Math.sin(angle-z)+r);
            robot.rightBackMotor.setPower(power*Math.cos(angle-z)+r);
            robot.leftBackMotor.setPower(-power*Math.sin(angle-z)+r);
            telemetry.addData("angle, z",  "%.2f, %.2f", angle, z);
            telemetry.addData("power",  "%.2f", power);
        } else { // use the old mode
            robot.leftFrontMotor.setPower(x+r);  // Set wheels equal to left stick //
            robot.rightFrontMotor.setPower(-y-r);  // direction plus amount of turn  //
            robot.rightBackMotor.setPower(x-r);
            robot.leftBackMotor.setPower(r-y);
        }

        // Use A and B to open and close the clamp
        if (gamepad1.b)
            clampOffset += CLAMP_SPEED;
        else if (gamepad1.a)
            clampOffset -= CLAMP_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clampOffset = Range.clip(clampOffset, -0.75, 0.75);
        robot.leftClamp.setPosition(robot.CLAMP_LEFT_OPEN + clampOffset);
        robot.rightClamp.setPosition(robot.CLAMP_RIGHT_OPEN - clampOffset);

        // Use R1 and L1 to move the front lift up and down
        if(gamepad1.left_bumper)
            robot.liftMotor.setPower(robot.LIFT_DOWN_POWER);
        else if (gamepad1.right_bumper)
            robot.liftMotor.setPower(robot.LIFT_UP_POWER);
        else
            robot.liftMotor.setPower(0.0);

        // Use R2 and L2 to move the back lift up and down
        if (gamepad1.left_trigger > 0.1)
            robot.backLiftMotor.setPower(robot.LIFT_UP_POWER);
        else if (gamepad1.right_trigger > 0.1)
            robot.backLiftMotor.setPower(robot.LIFT_DOWN_POWER);
        else
            robot.backLiftMotor.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("clamp",  "Offset = %.2f", clampOffset);
        telemetry.addData("left.x,y",  "%.2f, %.2f", x, y);
        telemetry.addData("right.rotation", "%.2f", r);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
