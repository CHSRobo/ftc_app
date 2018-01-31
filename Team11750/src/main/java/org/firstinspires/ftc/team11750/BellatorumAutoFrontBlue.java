/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team11750;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the Bellatorum hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Bellatorum: Front Blue", group="Bellatorum")
public class BellatorumAutoFrontBlue extends BellatorumAuto {

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        autonomousInit(); // Initialize the autonomous method

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        if (!robot.clampInstalled) telemetry.addData("Status","### Clamp disabled ###");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gyro.resetZAxisIntegrator(); // Reset the gyro

        robot.clampClose(); // Grab the glyph
        sleep(1000); // Wait one second
        liftUp(1); // Raise the lift in ft

        // Get the RelicRecoverVuMark location
        relicVuMark = getRelicRecoveryVuMark();

        displaceJewel(robot.COLOR_RED); // Knock off the jewel of this color

        move(robot.LEFT, 2.5, 1); // Move in feet
        turn(robot.RIGHT/6, 0.8); turn(robot.FORWARD, 0.8 ); // Wiggle off the platform
        move(robot.FORWARD, 0.33); // move away from the glyph box
        move(robot.RIGHT, 1.0, 0.2); // Move back to align with platform

        // Move the robot according to the relic VuMark
        double relicMove = 7.63/12; // Default to move in feet
        double turnAngle=-145.0; // Default turn angle
        if (relicVuMark == RelicRecoveryVuMark.LEFT) { turnAngle += -19.0; } // Turn a little further
        if (relicVuMark == RelicRecoveryVuMark.RIGHT) { relicMove += 8.63/12; } // 7.63" further
        move(robot.LEFT, relicMove); // Move left relicMove feet
        turn(turnAngle); // Turn left in degrees
        move(robot.FORWARD, 0.75); // Move forward in feet

        liftDown(0.6); // Lower the lift in ft
        robot.clampOpen(); // Drop the glyph
        move(robot.FORWARD, 0.65, 0.4); // Move forward in feet

        move(robot.BACK, 0.2); // Back up

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
