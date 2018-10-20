package org.firstinspires.ftc.team11383;

/**
 * Created by Walt on 11/13/17.
 */

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left Front drive motor:   "left_front_drive"
 * Motor channel:  Right Front drive motor:  "right_front_drive"
 * Motor channel:  Left Back drive motor:    "left_back_drive"
 * Motor channel:  Right Back drive motor:   "right_back_drive"
 * Motor channel:  Lift Drive motor:         "lift_arm"
 * Servo channel:  Servo to move left clamp: "left_hand"
 * Servo channel:  Servo to move right clamp:"right_hand"
 */
class HardwareFortissimus2
{
    /* Public OpMode members. */
    DcMotor  leftFrontMotor   = null; // runs in x direction //
    DcMotor  rightFrontMotor  = null; // runs in y direction //
    DcMotor  leftBackMotor    = null; // runs in y  direction //
    DcMotor  rightBackMotor   = null; // runs in x direction //
    DcMotor  reel   = null;
    DcMotor track = null;
    Servo    armleft   = null;
    Servo    armright  = null;
    Servo    color = null;
    Servo   colorh = null;
    ColorSensor c;

    final double ARM_LEFT_OPEN  =  0.4;
    final double ARM_RIGHT_OPEN = 0.7;
    final double ARM_LEFT_CLOSED  = 1.0;
    final double ARM_RIGHT_CLOSED = 0.0;
    final double LIFT_UP_POWER    =  0.25 ;
    final double LIFT_DOWN_POWER  = -0.25 ;
    final double LIFT_FEET_PER_SEC = 5;
    final double FORWARD_POWER = 0.6;
    final double FEET_PER_SEC = 4;
    final double MOVE_START_SECS = 0.1;
    final double TURN_POWER    = 0.1;
    final double FORWARD =0.0;
    final double LEFT = 90.0;
    final double RIGHT = -90.0;
    final double BACK = 180.0;
    final double AROUND = 180.0;
    final double DEGREES_PER_SEC = 340+.0;
    final double TURN_START_SECS = 0.2;
    final double ARM_UP = 0.18;
    final double ARM_DOWN = 115;
    final int COLOR_RED = 1;
    final int COLOR_BLUE = 2;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareFortissimus2(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.dcMotor.get("fl");
        rightFrontMotor  = hwMap.dcMotor.get("fr");
        leftBackMotor    = hwMap.dcMotor.get("bl");
        rightBackMotor   = hwMap.dcMotor.get("br");
        track   = hwMap.dcMotor.get("track");
        reel    = hwMap.dcMotor.get("reel");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        reel.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        armleft = hwMap.servo.get("al");
        armright = hwMap.servo.get("ar");
        armleft.setPosition(ARM_LEFT_CLOSED);
        armright.setPosition(ARM_RIGHT_CLOSED);
        color = hwMap.servo.get("ac");
        colorh = hwMap.servo.get("ach");
        color.setPosition(ARM_UP);
        colorh.setPosition(.5);

        // get a reference to our colorSensor
        c = hwMap.get(ColorSensor.class, "c");
    }

    // Stop the robot from moving
    void stopMoving() {
        leftBackMotor.setPower(0.0); // Stop
        rightFrontMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
    }

    // Start the robot turning in the angle direction at specified power
    void startRotate(double angle, double power) {
        double direction = 1.0; // The direction to turn

        if (angle < 0) { // If the angle is negative
            direction = -1; // Toggle the direction
            angle *= -1; // Make the angle positive
        }

        // Set all motors to turn in direction at power
        rightBackMotor.setPower(power * direction);
        rightFrontMotor.setPower(power * direction);
        leftFrontMotor.setPower(power * direction);
        leftBackMotor.setPower(power * direction);
    }

    // Start the robot moving in the direction specified by angle (relative to the robot front)
    void startMovingInDirection(double angle, double power){
        rightFrontMotor.setPower(-power * Math.cos((Math.PI / 180) * angle));
        leftBackMotor.setPower(power * Math.cos((Math.PI / 180) * angle));
        leftFrontMotor.setPower(power * Math.sin((Math.PI / 180) * angle));
        rightBackMotor.setPower(-power * Math.sin((Math.PI / 180) * angle));
    }

    long timer = 0;
    boolean FlipDrive = true;

    // Set the clamp to the specified open angle
    void clampOpen(double angle){
        armleft.setPosition(ARM_LEFT_CLOSED - angle/2/180);
        armright.setPosition(ARM_RIGHT_CLOSED + angle/2/180);
    }
    void clampOpen() {clampOpen(180);} // Open the clamp all the way
    void clampClose() {clampOpen(30);} // Close the clamp on a glyph

    // Set the color arm to the specified down angle from 0 degrees straight up, 100 degrees down
    void armPosition(double angle) {
        color.setPosition(ARM_UP + angle/180);
    }
    void armDown() {armPosition(ARM_DOWN);} // Drop the arm to the ground
    void armUp() {armPosition(0);} // Raise the arm to the bot

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
