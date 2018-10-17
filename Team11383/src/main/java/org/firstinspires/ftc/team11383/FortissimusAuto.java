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

package org.firstinspires.ftc.team11383;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the Bellatorum hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class FortissimusAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private HardwareFortissimus2 robot   = new HardwareFortissimus2();   // Use Bellatorum's hardware
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark relicVuMark = RelicRecoveryVuMark.UNKNOWN;

    void initVuforia(){
        int cameraMonitorViewId;

        // Use CHS Robotics license key
        parameters.vuforiaLicenseKey = "AfOu+xX/////AAAAGdsYKU+bz0Fnv1XlcuaTiqUXVLGVTLZI6iw2Ddd34qXAIdi6IjqLFqG7Tm1uGNvfW29lkxuh2jF47MydTZX9AdADaEW2NuPtfFpGDQQd9wto5MIjzJHIWnY4aBGY8zDtePEHX68Sez31rq3IfGuKIQBa/Ewsl8obrkMQLlUvdLYNVRLvQVnvp9beui5vF3YU+gGKEs76eN27tF40Uq+u3SqRqpbC9W+2p33xHIdyxmJynd4OYF9PQjdB0oGajsRBpZSVjD+mwtBYynshpj3ay2coXvzBO250/MkGp7ZEXdHC8C0uYqz/jXQaBjuLGdBBVUukBGLTgSqLO3Q33SI5WCykF8G05G+5YmWJ2KDcp/ze";

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        // Use the front camera on the robot controller phone
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

     RelicRecoveryVuMark getRelicRecoveryVuMark() {

        relicTrackables.activate();

        // Look for a bit to see the VuMark
        runtime.reset();
        while (runtime.seconds() < 1) {
            if (!opModeIsActive()) {robot.stopMoving(); break;} // Stop and return
            /*
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            relicVuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (relicVuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", relicVuMark);
                telemetry.update();
                return relicVuMark; // Return now. No need to wait longer
            }
            else {
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
        }
        return relicVuMark;
    }

    private void turn(double angle, double power) {

        if (angle==0) return; // Return immediately for 0 degree turn
        robot.startRotate(angle, power); // Start rotating in the angle direction at power
        if (angle < 0) { angle *= -1; } // If the angle is negative make it positive

        // Turn long enough to make the angle
        runtime.reset();
        while (runtime.seconds() < angle/robot.DEGREES_PER_SEC/power + robot.TURN_START_SECS) {
            telemetry.addData("Turning: ", "%2.5f secs Elapsed", runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        robot.stopMoving();
    }
    void turn(double angle) {turn(angle, robot.TURN_POWER);} // Overload with default power

    private void move(double angle, double distance, double power){
        robot.startMovingInDirection(angle, power); // Start moving in the right direction

        // Run long enough to make the distance
        runtime.reset();
        while (runtime.seconds() < distance/robot.FEET_PER_SEC/power) {
            telemetry.addData("Moving: ", "%2.5f deg, %2.5f ft, %2.5f secs Elapsed",
                    angle, distance, runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        robot.stopMoving();
    }
    void move(double angle, double distance){ // Overload with default power
        move(angle, distance, robot.FORWARD_POWER);
    }

    private void lift(double directionPower, double distance){
       // if(!robot.clampInstalled) return;
        robot.reel.setPower(directionPower);
        if (directionPower<0)directionPower*=-1; // Make sure the power positive
        runtime.reset();
        while (runtime.seconds() < distance / robot.LIFT_FEET_PER_SEC/directionPower) {
            telemetry.addData("Lift", "Time: %2.5f secs Elapsed", runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        robot.reel.setPower(0.0);
    }
    void liftUp(double distance) { lift(robot.LIFT_UP_POWER, distance);}
    void liftDown(double distance) { lift(robot.LIFT_DOWN_POWER, distance);}

    void displaceJewel(int color){
        double turnAngle = 0;
        robot.armDown(); // Drop the color sensor arm

        runtime.reset();
        while (runtime.seconds() < 1) {
            telemetry.addData("Clear", robot.c.alpha());
            telemetry.addData("Red  ", robot.c.red());
            telemetry.addData("Green", robot.c.green());
            telemetry.addData("Blue ", robot.c.blue());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        // Displace the blue color ball
        if (robot.c.blue() >= 1) turnAngle += 20;
        if (robot.c.red() >= 1) turnAngle -= 20;

        // Turn the other way to displace thr
        if(color == robot.COLOR_RED) turnAngle*=-1; // Turn the other way

        turn(turnAngle); // Turn to knock off the jewel
        robot.armUp();   // Raise the arm
        turn(-turnAngle);// Turn back
    }
    void redTeamDisplaceJewel(){ displaceJewel(robot.COLOR_BLUE);}
    void blueTeamDisplaceJewel() {displaceJewel(robot.COLOR_RED);}

    @Override public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "This is a template Autonomous. Override");    //
        telemetry.update();

        // Initialize the Vuforia capability
        initVuforia();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Get the RelicRecoverVuMark location
        getRelicRecoveryVuMark();
    }
}
