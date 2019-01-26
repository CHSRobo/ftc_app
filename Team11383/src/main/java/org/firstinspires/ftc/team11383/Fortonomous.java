// Fortonomous.java
// Autonomous OpMode
// Will contain more complex full autonomous commands

package org.firstinspires.ftc.team11383;

/**
 * Created by Will Archer on 11/26/2018
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Fortonomous")
public class Fortonomous extends StolenAutoByrd {
    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void runOpMode() throws InterruptedException {
    
    /*
    * $ next to complete items
    *
    * Current Plan:
    * Use Lead Screw, Land from Lander
    * Use Gyro, Move to Crater
    */

    robot.init(hardwareMap);
        autonomousInit();
        waitForStart();
        autonomousStart();
        if (!isStopRequested()) {
            lift(false);
            robot.hook.setPosition(0);
            sleep(1000);
            moveForward(6000,.5);
            robot.leftArm.setPosition(0);
            robot.rightArm.setPosition(0);
            if(robot.leftColor.green()>50) {
                robot.leftFrontMotor.setPower(-1);
                robot.rightFrontMotor.setPower(1);
                robot.leftBackMotor.setPower(1);
                robot.rightBackMotor.setPower(-1);
                sleep(1000);
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
            }
            else if(robot.rightColor.green()>50) {
                robot.leftFrontMotor.setPower(1);
                robot.rightFrontMotor.setPower(-1);
                robot.leftBackMotor.setPower(-1);
                robot.rightBackMotor.setPower(1);
                sleep(1000);
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
            }
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(1);
            moveForward(2000,.5);
            stopMoving();
        }

    }
}
