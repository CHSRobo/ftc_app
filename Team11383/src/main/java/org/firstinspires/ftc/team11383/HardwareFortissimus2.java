package org.firstinspires.ftc.team11383;

/**
 * Created by Walt on 11/13/17.
 */

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
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

    public DcMotor  leftFrontMotor   = null; // runs in x direction //
    public DcMotor  rightFrontMotor  = null; // runs in y direction //
    public DcMotor  leftBackMotor    = null; // runs in y  direction //
    public DcMotor  rightBackMotor   = null; // runs in x direction //
    public DcMotor  leftActuator     = null; // Extends and Retracts //
    public DcMotor  rightActuator    = null; // Lifts and Lowers the Bot //
    public DcMotor  mineralArm       = null; // Lifts and Drops Minerals //
    public Servo    relic            = null; // Knocks the relic //
    public Servo    rightArm         = null; // Color Sensor Arm //
    public Servo    leftArm          = null; // Color Sensor Arm //
    public Servo    hook             = null; // Moves the hook //

    public ColorSensor leftColor;
    public ColorSensor rightColor;
    public GyroSensor gyro;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareFortissimus2(){

    }

    // Declare misc variables
    static double WHEEL_CIRCUMFERENCE = 4;
    static double PULSES_PER_REVOLUTION = 280;

    static double INPUT_TIMER = 0;

    static double DEFAULT_TURN_SPEED = 10;//In degrees per second
    static double DEFAULT_MOVE_SPEED = 6;//In inches per second

    static double MOVE_LEFT  = 180;
    static double MOVE_FORE  = 90;
    static double MOVE_AFT   = 270;
    static double MOVE_RIGHT = 0;

    static double TURN_ERROR = 1;

    static boolean LIFT_UP = true;
    static boolean LIFT_DOWN = false;

    static boolean UNHOOK = false;
    static boolean HOOK   = true;

    static double  HOOK_CLOSED = 0;
    static double  HOOK_OPEN   = 1;

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        HardwareMap hwMap;
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = ahwMap.get(DcMotor.class,"fl");
        rightFrontMotor  = ahwMap.get(DcMotor.class,"fr");
        leftBackMotor    = ahwMap.get(DcMotor.class,"bl");
        rightBackMotor   = ahwMap.get(DcMotor.class,"br");
        rightActuator    = ahwMap.get(DcMotor.class,"ra");
        leftActuator     = ahwMap.get(DcMotor.class,"la");
        mineralArm       = ahwMap.get(DcMotor.class,"ma");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to all to FORWARD
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightActuator.setPower(0);
        leftActuator.setPower(0);
        mineralArm.setPower(0);

        // Set drive and arm motors to run without encoders.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set actuator motors to run with encoders.
        rightActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Define and initialize ALL installed servos.
        relic = ahwMap.servo.get("rc");
        rightArm = ahwMap.servo.get("ra");
        leftArm = ahwMap.servo.get("la");
        hook = ahwMap.servo.get("hk");

        // Define Sensors
        rightColor = hwMap.get(ColorSensor.class, "rcolor");
        leftColor = hwMap.get(ColorSensor.class, "lcolor");
        gyro = hwMap.get(GyroSensor.class,"gyro");

    }



/* // Old Omni functions

    // Start the robot turning in the angle direction at specified power
    void rotate(double angle, double power) {
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
    void move(double angle, double power){
        rightFrontMotor.setPower(-power * Math.cos((Math.PI / 180) * angle));
        leftBackMotor.setPower(power * Math.cos((Math.PI / 180) * angle));
        leftFrontMotor.setPower(power * Math.sin((Math.PI / 180) * angle));
        rightBackMotor.setPower(-power * Math.sin((Math.PI / 180) * angle));
    } */

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
