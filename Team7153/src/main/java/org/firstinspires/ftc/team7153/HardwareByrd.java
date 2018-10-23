package org.firstinspires.ftc.team7153;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 */
class HardwareByrd
{
    /* Public OpMode members. */
    DcMotor  frontRight        = null;
    DcMotor  frontLeft         = null;
    DcMotor  backRight         = null;
    DcMotor  backLeft          = null;
    DcMotor  lift              = null;

    ModernRoboticsI2cGyro gyro         = null;

    Servo hook;

    static double WHEEL_CIRCUMFERENCE = 4;
    static double PULSES_PER_REVOLUTION = 280;

    static double INPUT_TIMER = 0;

    static double DEFAULT_TURN_SPEED = 10;//In degrees per second
    static double DEFAULT_MOVE_SPEED = 12;//In inches per second

    static double MOVE_LEFT  = 180;
    static double MOVE_FORE  = 90;
    static double MOVE_AFT   = 270;
    static double MOVE_RIGHT = 0;

    static double TURN_ERROR = 1;

    static boolean LIFT_UP = true;
    static boolean LIFT_DOWN = false;

    /* Constructor */
    HardwareByrd() {

    }
    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        HardwareMap hwMap;

        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRight = ahwMap.get(DcMotor.class, "fr");
        frontLeft  = ahwMap.get(DcMotor.class, "fl");
        backRight  = ahwMap.get(DcMotor.class, "br");
        backLeft   = ahwMap.get(DcMotor.class, "bl");

        lift   = ahwMap.get(DcMotor.class, "lift");
        hook   = ahwMap.get(Servo.class, "hook");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);
 
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift. setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set all motors to zero power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        lift.setPower(0);

        // Define Sensors
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
    }
 }

