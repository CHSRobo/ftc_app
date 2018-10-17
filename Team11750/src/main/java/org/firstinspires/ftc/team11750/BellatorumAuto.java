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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the Bellatorum hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you have encoders on the wheels,
 *
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: startMovingEncoder
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class BellatorumAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBellatorum robot   = new HardwareBellatorum();   // Use Bellatorum's hardware
    private ElapsedTime runtime = new ElapsedTime();
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.008;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark relicVuMark = RelicRecoveryVuMark.UNKNOWN;

    void log(String update){
        telemetry.addLine(update);
        telemetry.update();
    }

    void initVuforia(){
        int cameraMonitorViewId;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Use CHS Robotics license key
        parameters.vuforiaLicenseKey = "AfOu+xX/////AAAAGdsYKU+bz0Fnv1XlcuaTiqUXVLGVTLZI6iw2Ddd34qXAIdi6IjqLFqG7Tm1uGNvfW29lkxuh2jF47MydTZX9AdADaEW2NuPtfFpGDQQd9wto5MIjzJHIWnY4aBGY8zDtePEHX68Sez31rq3IfGuKIQBa/Ewsl8obrkMQLlUvdLYNVRLvQVnvp9beui5vF3YU+gGKEs76eN27tF40Uq+u3SqRqpbC9W+2p33xHIdyxmJynd4OYF9PQjdB0oGajsRBpZSVjD+mwtBYynshpj3ay2coXvzBO250/MkGp7ZEXdHC8C0uYqz/jXQaBjuLGdBBVUukBGLTgSqLO3Q33SI5WCykF8G05G+5YmWJ2KDcp/ze";

        // Use the front camera on the robot controller phone
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

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
                sleep(500);
                return relicVuMark; // Return now. No need to wait longer
            }
            else {
                telemetry.addData("VuMark", "not visible: %2.1f secs", runtime.seconds());
                telemetry.update();
            }
        }
        return relicVuMark;
    }

    void oldTurn(double angle, double power) {

        if (angle==0) return; // Return immediately for 0 degree turn
        robot.startRotate(angle, power); // Start rotating in the angle direction at power
        if (angle < 0) { angle *= -1; } // If the angle is negative make it positive

        // Turn long enough to make the angle
        runtime.reset();
        while (runtime.seconds() < angle/robot.DEGREES_PER_SEC/power + robot.TURN_START_SECS) {
            telemetry.addData("Turning: ", "%2.1f secs Elapsed", runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        robot.stopMoving();
    }
    void turn(double angle, double power) { gyroTurn(power, angle);}
    void turn(double angle) {turn(angle, robot.TURN_POWER);} // Overload with default power

    void oldMove(double angle, double distance, double power){
        log("Start moving...");
        robot.startMovingEncoder(angle, distance, power); // Start moving in the right direction

        // Run long enough to make the distance + 1 sec, then timeout
        runtime.reset();
        while (robot.motorsBusy() && runtime.seconds() < distance/robot.FEET_PER_SEC/power + 2) {
            telemetry.addData("Moving: ", "%2.1f deg, %2.2f ft, %2.1f secs Elapsed",
                    angle, distance, runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        log("Stop moving...");
        robot.stopMoving();
        robot.setupEncoders();
    }
    void move(double angle, double distance, double power){
        gyroDrive(power, distance, angle);
    }

    void move(double angle, double distance){ // Overload with default power
        move(angle, distance, robot.FORWARD_POWER);
    }

    private void lift(double directionPower, double distance){
        if(!robot.clampInstalled) return;
        robot.liftMotor.setPower(directionPower);
        if (directionPower<0)directionPower*=-1; // Make sure the power positive
        runtime.reset();
        while (runtime.seconds() < distance / robot.LIFT_FEET_PER_SEC/directionPower) {
            telemetry.addData("Lift", "Time: %2.3f secs Elapsed", runtime.seconds());
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        robot.liftMotor.setPower(0.0);
    }
    void liftUp(double distance) { lift(robot.LIFT_UP_POWER, distance);}
    void liftDown(double distance) { lift(robot.LIFT_DOWN_POWER, distance);}

    void displaceJewel(int color){
        double turnAngle = 0;
        double red=0, blue=0;
        double initialTurn=3;
        log("ArmDown");
        robot.armDown(); // Drop the color sensor arm
        sleep(100); // Wait for the arm to drop

        turn(initialTurn); // Get close to the jewel

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            blue+=robot.colorSensor.blue(); // Add up the blue readings
            red+=robot.colorSensor.red(); // Add up the red readings
            telemetry.addData("Clear", robot.colorSensor.alpha());
            telemetry.addData("Red  ", red);
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.addData("Blue ", blue);
            telemetry.update();
            if (!opModeIsActive()) {robot.stopMoving(); return;} // Stop and return
        }
        // Displace the blue color ball
        if (blue > red) turnAngle += 12;
        else if (red > blue) turnAngle -= 12;

        // Turn the other way to displace red
        if(color == robot.COLOR_RED) turnAngle*=-1; // Turn the other way

        log("Displacing jewel...");
        turn(turnAngle-initialTurn); // Turn to knock off the jewel
        robot.armUp();   // Raise the arm
        turn(0);// Turn back
        log("Jewel displaced!");
    }
    void redTeamDisplaceJewel(){ displaceJewel(robot.COLOR_BLUE);}
    void blueTeamDisplaceJewel() {displaceJewel(robot.COLOR_RED);}

    void autonomousInit(){
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        // Initialize the Vuforia capability
        initVuforia();
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        autonomousInit();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "This is a template Autonomous. Override");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gyro.resetZAxisIntegrator();

        // Get the RelicRecoverVuMark location
        getRelicRecoveryVuMark();
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {
        double  error;
        double  steer;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            log("Start moving...");
            robot.startMovingEncoder(angle, distance, speed); // Start moving in the right direction

            // Run long enough to make the distance + 1 sec, then timeout
            runtime.reset();
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && robot.motorsBusy()
                    && runtime.seconds() < distance/robot.FEET_PER_SEC/speed + 0.5) {

                // adjust direction and speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                robot.startMovingInDirection(angle-steer, speed);

                // Display drive status for the driver.
                telemetry.addData("Angle/Speed",   "%5.2f:%5.2f",  angle, speed);
                telemetry.addData("Err/St/New Angle",  "%5.1f/%5.1f, %5.2f",
                        error, steer, angle-steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(),
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Runtime", "%5.3f", runtime.seconds());
                telemetry.update();
            }

            // Stop all motion;
            robot.stopMoving();

            // Turn off RUN_TO_POSITION
            robot.setupEncoders();
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.stopMoving();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            robot.stopMoving();
            onTarget = true;
        }
        else {
            steer = getSteer(error, P_TURN_COEFF);
            robot.startRotate(error, steer);
        }

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle + gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(Math.abs(error * PCoeff), -1, 1);
    }

}
