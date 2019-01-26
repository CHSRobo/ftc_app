// Fortonomous.java
// Autonomous OpMode
// Will contain more complex full autonomous commands

package org.firstinspires.ftc.team11383;

/**
 * Created by Will Archer on 11/26/2018
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.team11383.HardwareFortissimus2.DEFAULT_TURN_SPEED;
import static org.firstinspires.ftc.team11383.HardwareFortissimus2.INPUT_TIMER;
import static org.firstinspires.ftc.team11383.HardwareFortissimus2.PULSES_PER_REVOLUTION;
import static org.firstinspires.ftc.team11383.HardwareFortissimus2.TURN_ERROR;

/*
@Autonomous(name="FunctonomousVoid")
public class FunctonomousVoid extends LinearOpMode {
    private HardwareFortissimus2 robot = new HardwareFortissimus2();
    private double imaginaryAngle=0;         //Sets the robot's initial angle to 0


    private ElapsedTime runTime = new ElapsedTime();

    void autonomousInit(){
        //Hardware//
        robot.init(hardwareMap);
    }

    void autonomousStart() {
        //Reset the timer to 0
        runTime.reset();
    }

    // Stop the robot from moving
    void stopMoving() {
        robot.leftBackMotor.setPower(0); // Stop
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }



    void move(double DISTANCE, double SPEED, double DIRECTION) throws InterruptedException {
        if(isStopRequested()){
            return;
        }
        turn(DIRECTION,DEFAULT_TURN_SPEED);

        double deltaDistance;
        double deltaDisplacement = 0;
        double pastTime = getRuntime();

        double speedIntegral = 0;
        double angleIntegral = 0;
        double angleError = 0;

        double pastWheelAverage = (robot.leftFrontMotor.getCurrentPosition()+robot.rightFrontMotor.getCurrentPosition()+robot.rightBackMotor.getCurrentPosition()+robot.leftBackMotor.getCurrentPosition())/4;
        while(deltaDisplacement<DISTANCE){
            double deltaTime = getRuntime()-pastTime;
            pastTime = getRuntime();


            deltaDistance = ((robot.leftFrontMotor.getCurrentPosition()+robot.rightFrontMotor.getCurrentPosition()+robot.rightBackMotor.getCurrentPosition()+robot.leftBackMotor.getCurrentPosition())/4-pastWheelAverage);
            pastWheelAverage = deltaDistance;
            deltaDistance = 4*deltaDistance/PULSES_PER_REVOLUTION;
            double inchesPerSecond = deltaDistance/deltaTime;

            //SPEED PI
            double speedError = SPEED - inchesPerSecond;
            speedIntegral += (speedError * deltaTime);
            double speedOutput = (1/4 * speedError);// + (1 * speedIntegral);
            //

            //ANGLE PI
            angleError += Math.sin(DIRECTION)*deltaDistance;
            angleIntegral += (angleError * deltaTime);
            double angleOutput = (1/90 * angleError);// + (1 * angleIntegral);
            //

            robot.leftFrontMotor.setPower(speedOutput+angleOutput);
            robot.rightFrontMotor.setPower(speedOutput-angleOutput);
            robot.leftBackMotor.setPower(speedOutput+angleOutput);
            robot.rightBackMotor.setPower(speedOutput-angleOutput);

            deltaDisplacement += Math.cos(DIRECTION)*deltaDistance;
            telemetry.addData("speedError:      ", speedError);
            telemetry.addData("speedIntegral:   ", speedIntegral);
            telemetry.addData("speedOutput:     ", speedOutput);
            telemetry.addData("angleError:      ", angleError);
            telemetry.addData("angleIntegral:   ", angleIntegral);
            telemetry.addData("angleOutput:     ", angleOutput);
            telemetry.addData("inchesPerSecond: ", inchesPerSecond);
            telemetry("Move", "Main Loop");
            if(isStopRequested()){
                return;
            }
        }
        stopMoving();
    }

    private void turn(double angle, double speed) throws InterruptedException {
        if (isStopRequested()) {
            return;
        }
        //Sets the angle that the robot is supposed to be in to the angle argument
        imaginaryAngle = angle;

        float pastAngle = gyroZ(), currentAngle = gyroZ(), deltaAngle = 0;
        double pastTime = getRuntime(), currentTime = getRuntime(), deltaTime = 0;
        double anglePerSecond;
        double speedIntegral = 0;
        //While the angel is > the gyroscope+TURN_ERROR or < the gyroscope-TURN_ERROR
        resetTimer();
        while ((gyroZ() < angle - TURN_ERROR || gyroZ() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && gyroZ() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && gyroZ() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5 > getRuntime()) {
            if (isStopRequested()) {
                stopMoving();
                return;
            }

            currentAngle = gyroZ();
            currentTime = getRuntime();
            deltaAngle = Math.abs(currentAngle-pastAngle);
            deltaTime = currentTime-pastTime;
            anglePerSecond = deltaAngle/deltaTime;
            pastAngle=currentAngle;
            pastTime=currentTime;
            double speedError = speed - anglePerSecond;
            speedIntegral += (speedError * deltaTime);
            double speedOutput = (1/8 * speedError) + (.001 * speedIntegral);

            //Checks to see if turning left or right
            if ((angle > gyroZ() && angle < gyroZ() + 181) || (angle < gyroZ() - 180)) {
                robot.frontLeft.setPower(-speedOutput);
                robot.frontRight.setPower(speedOutput);
                robot.backLeft.setPower(-speedOutput);
                robot.backRight.setPower(speedOutput);
            } else {
                robot.frontLeft.setPower(speedOutput);
                robot.frontRight.setPower(-speedOutput);
                robot.backLeft.setPower(speedOutput);
                robot.backRight.setPower(-speedOutput);
            }

            telemetry.addData("Function: ", "Turn");
            telemetry.addData("Target Angle:  ", angle);
            telemetry.addData("Current Speed: ", speed);
            telemetry("Turn" , "Main Loop");
        }
        stopMoving();
        sleep(100);
        if ((gyroZ() < angle - TURN_ERROR || gyroZ() > angle + TURN_ERROR) && (angle - TURN_ERROR <= -1 && gyroZ() != 360 - TURN_ERROR || angle - TURN_ERROR > -1) && (angle + TURN_ERROR >= 360 && gyroZ() > TURN_ERROR - 1 || angle + TURN_ERROR < 360) && INPUT_TIMER + 5 > getRuntime()) {
            straighten();
        }
    }


}
*/
