package org.firstinspires.ftc.team7153;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    DcMotor  linearSlide       = null;
    DcMotor  dumpRotate        = null;

    Servo    hook           = null;

    CRServo  intake         = null;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

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

    /* Constructor */
    HardwareByrd() {

    }
    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        HardwareMap hwMap;

        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Define and Initialize Motors
        frontRight = ahwMap.get(DcMotor.class, "fr");
        frontLeft  = ahwMap.get(DcMotor.class, "fl");
        backRight  = ahwMap.get(DcMotor.class, "br");
        backLeft   = ahwMap.get(DcMotor.class, "bl");

        linearSlide = ahwMap.get(DcMotor.class, "linearSlide");
        dumpRotate  = ahwMap.get(DcMotor.class, "dumpRotate");

        lift   = ahwMap.get(DcMotor.class, "lift");
        hook   = ahwMap.get(Servo.class, "hook");

        intake = ahwMap.get(CRServo.class, "intake");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        dumpRotate.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.FORWARD);
 
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dumpRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set all motors to zero power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        linearSlide.setPower(0);
        dumpRotate.setPower(0);

        intake.setPower(0);

        lift.setPower(0);
        hook.setPosition(HOOK_CLOSED);

        // Define Sensors

        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
 }

