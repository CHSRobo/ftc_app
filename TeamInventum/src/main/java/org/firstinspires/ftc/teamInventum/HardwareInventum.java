//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamInventum;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
class HardwareInventum
{
    /* OpMode members. */
	/*no change needed for inventum*/
    DcMotor  leftMotor   = null;
    DcMotor  rightMotor  = null;
    DcMotor  backClamp   = null;
    DcMotor  frontClamp  = null;
    Servo    leftClaw    = null;
    Servo    rightClaw   = null;
    Servo    hammer      = null;

    ModernRoboticsI2cColorSensor color = null;
    ModernRoboticsI2cGyro        gyro  = null;


    static final double TURN_FORWARDS = 0;
    static final double TURN_LEFT     = 90;
    static final double TURN_RIGHT    = 270;
    static final double TURN_BACK     = 180;

    static final double LEFT_CLAMP_CLOSE  =  0;
    static final double RIGHT_CLAMP_CLOSE =  1;
    static final double LEFT_CLAMP_OPEN   =  0.4;
    static final double RIGHT_CLAMP_OPEN  =  0.6;

    static final double HAMMER_DOWN       =  1.0;
    static final double HAMMER_UP         =  .3;

    static final double TURN_ERROR        =  1;
    static double       INPUT_TIMER       =  0;

    static final boolean RED  = true;
    static final boolean BLUE = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareInventum(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        frontClamp    = hwMap.dcMotor.get("front_clamp");
        backClamp    = hwMap.dcMotor.get("back_clamp");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        frontClamp.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backClamp.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        frontClamp.setPower(0);
        backClamp.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontClamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backClamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw = hwMap.servo.get("left_hand");
        rightClaw = hwMap.servo.get("right_hand");
        hammer = hwMap.servo.get("hammer");
        leftClaw.setPosition(LEFT_CLAMP_OPEN);
        rightClaw.setPosition(RIGHT_CLAMP_OPEN);
        hammer.setPosition(HAMMER_UP);

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        color = hwMap.get(ModernRoboticsI2cColorSensor.class, "color");
        color.enableLed(false);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    void waitForTick(long periodMs) {

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

