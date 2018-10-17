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

package org.firstinspires.ftc.team7153;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
    DcMotor  dump              = null;

    DcMotor  idolZ             = null;
    DcMotor  idolY             = null;

    CRServo  intakeFrontLeft   = null;
    CRServo  intakeFrontRight  = null;
    CRServo  intakeBackLeft    = null;
    CRServo  intakeBackRight   = null;

    Servo    hammerY           = null;
    Servo    hammerX           = null;

    //Servo    plate             = null;

    Servo    grabber           = null;

    Servo    intakeLatch       = null;
    Servo    blockPusher       = null;

    ModernRoboticsI2cGyro gyro         = null;
    ModernRoboticsI2cColorSensor color = null;
    ModernRoboticsI2cRangeSensor cubbyRange = null;
    ModernRoboticsI2cRangeSensor blockRange = null;

    private static final double IDOL_Z_START_POSITION =  0;
    static final int    IDOL_Z_DELTA_POSITION = 60;

    static double        INTAKE_SPEED      =     0;
    static double        INTAKE_OFFSET     =     0;
    //static boolean       IS_BLOCK_GRAB     = false;
    /*static final double  LEFT_CLAMP_CLOSE  =     1;
    static final double  RIGHT_CLAMP_CLOSE =     0;
    static final double  TOP_CLAMP_CLOSE   =   -.2;
    static final double  LEFT_CLAMP_OPEN   =    .6;
    static final double  RIGHT_CLAMP_OPEN  =    .4;
    static final double  TOP_CLAMP_OPEN    =     1;
    private static final double  LEFT_CLAMP_INIT   =    .4;
    private static final double  RIGHT_CLAMP_INIT  =    .6;
    private static final double  TOP_CLAMP_INIT    =    -1;*/

    static boolean       IS_LIFT   =  false;
    static final double  LIFT_DOWN =  0;
    static final double  LIFT_UP   =  -10*240/(.4*Math.PI);//(InchesToTravel*1440/InchesPerRevolution)

    static final int DUMP_INTAKE   = -1;
    static final int DUMP_EXPEL    =  1;
    static final int DUMP_INACTIVE =  0;
    
    static boolean       IS_DUMP         = false;
    static final int     DUMP_UP         = 4*100; //4 Pulses per Degree
    static final int     DUMP_DOWN       =    0;

    static final double  HAMMER_DOWN    =   1;
    static final double  HAMMER_UP      =   .28;
    static final double  HAMMER_LEFT    =    .6;
    static final double  HAMMER_RIGHT   =    .2;
    static final double  HAMMER_CENTER  =    .4;

    static boolean       IS_PLATE        = false;
    static final double  PUSH_PLATE_DOWN =     0;
    static final double  PUSH_PLATE_UP   =     1;

    static final double  IDOL_CLAMP_OPEN   =     0;
    static final double  IDOL_CLAMP_AJAR   =    .3;
    static final double  IDOL_CLAMP_CLOSED =     1;

    static boolean       IS_BLOCK_PUSH = false;
    static final double  BLOCK_PUSH    = .15;
    static final double  BLOCK_NO_PUSH = 1;

    static boolean       IS_LATCH_RELEASE = false;
    static final double  LATCH_LOCKED     = .03;
    static final double  LATCH_UNLOCKED   = 1;

    static double INPUT_TIMER = 0;
    static double INPUT_TIMER_2 = 0;
    static boolean GYRO_MOVE  = false;

    static final boolean LEFT  = false;
    static final boolean RIGHT = true;

    static final boolean RED  = false;
    static final boolean BLUE = true;

    static final double MOVE_BACKWARDS = 270;
    static final double MOVE_FORWARDS  = 90;
    static final double MOVE_LEFT      = 180;
    static final double MOVE_RIGHT     = 0;

    static final boolean BACKWARDS = false;
    static final boolean FORWARDS  = true;

    static final double TURN_FORWARDS = 0;
    static final double TURN_LEFT     = 90;
    static final double TURN_RIGHT    = 270;
    static final double TURN_BACK     = 180;

    static final int TURN_ERROR = 3;

    static double DEFAULT_MOVE_SPEED = .3;
    static double DEFAULT_TURN_SPEED = .26;

    static boolean SLOT_1 = false;
    static boolean SLOT_2 = false;
    static boolean SLOT_3 = false;
    static double  OFFSET = 0;


    static boolean VUFORIA_ENABLED  = true;
    static boolean VUFORIA_DISABLED = false;

    /* Constructor */
    HardwareByrd(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        HardwareMap hwMap;

        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRight = hwMap.get(DcMotor.class, "fr");
        frontLeft  = hwMap.get(DcMotor.class, "fl");
        backRight  = hwMap.get(DcMotor.class, "br");
        backLeft   = hwMap.get(DcMotor.class, "bl");

        lift       = hwMap.get(DcMotor.class, "lift");
        dump       = hwMap.get(DcMotor.class, "dump");

        idolZ      = hwMap.get(DcMotor.class, "idolZ");
        idolY      = hwMap.get(DcMotor.class, "idolY");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        dump.setDirection(DcMotor.Direction.REVERSE);
        idolZ.setDirection(DcMotor.Direction.FORWARD);
        idolY.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        idolZ.setTargetPosition((int)IDOL_Z_START_POSITION);
        dump.setTargetPosition(DUMP_DOWN);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dump.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idolZ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idolY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        idolZ.setPower(1);
        idolY.setPower(0);
        dump.setPower(.1);
        lift.setPower(.5);

        // Define and initialize ALL installed servos.
/*        armL = hwMap.servo.get("armL");
        armR = hwMap.servo.get("armR");
        armT = hwMap.servo.get("armT");*/

        intakeFrontLeft = hwMap.crservo.get("intakeFrontLeft");
        intakeFrontRight= hwMap.crservo.get("intakeFrontRight");
        intakeBackLeft  = hwMap.crservo.get("intakeBackLeft");
        intakeBackRight = hwMap.crservo.get("intakeBackRight");

        intakeFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        hammerY = hwMap.servo.get("hammerY");
        hammerX = hwMap.servo.get("hammerX");

        //plate = hwMap.servo.get("plate");

        grabber = hwMap.servo.get("grabber");

        intakeLatch = hwMap.servo.get("intakeLatch");
        blockPusher = hwMap.servo.get("blockPusher");

/*        armL.setPosition(LEFT_CLAMP_INIT);
        armR.setPosition(RIGHT_CLAMP_INIT);
        armT.setPosition(TOP_CLAMP_INIT);*/
        hammerY.setPosition(HAMMER_UP);
        hammerX.setPosition(HAMMER_CENTER);
        //plate.setPosition(PUSH_PLATE_UP);
        grabber.setPosition(IDOL_CLAMP_OPEN);
        intakeLatch.setPosition(LATCH_LOCKED);
        blockPusher.setPosition(BLOCK_PUSH);

        intakeFrontLeft.setPower(0);
        intakeFrontRight.setPower(0);
        intakeBackLeft.setPower(0);
        intakeBackRight.setPower(0);

        // Define Sensors
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        color = hwMap.get(ModernRoboticsI2cColorSensor.class, "color");
        cubbyRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "cubbyRange");
        blockRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "blockRange");

        color.enableLed(false);

    }
 }

