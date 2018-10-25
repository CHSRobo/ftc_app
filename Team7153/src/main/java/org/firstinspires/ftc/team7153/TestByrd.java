package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;	
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;	
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;
import static org.firstinspires.ftc.team7153.HardwareByrd.PULSES_PER_REVOLUTION;

@TeleOp(name="TestByrd")
public class TestByrd extends OpMode{
     private HardwareByrd robot = new HardwareByrd();

    double pastWheelAverage = 0;
    double deltaDistance;
    double deltaDisplacement = 0;
    double pastTime = getRuntime();

    double speedIntegral = 0;
    double angleIntegral = 0;
    double angleError = 0;
    double SPEED = 6;
    double P = 0, I = 0;
    double inputTimer = 0;
    @Override
    public void init() {
    }	
    @Override	
    public void loop() {
        if(inputTimer+100<getRuntime()) {
            if (gamepad1.dpad_up) {
                P += .1;
                inputTimer = getRuntime();
            } else if (gamepad1.dpad_down) {
                P -= .1;
                inputTimer = getRuntime();
            } else if (gamepad1.dpad_right) {
                I += .1;
                inputTimer = getRuntime();
            } else if (gamepad1.dpad_left) {
                I -= .1;
                inputTimer = getRuntime();
            }
        }
        double deltaTime = getRuntime()-pastTime;
        pastTime = getRuntime();
        deltaDistance = ((robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+robot.backRight.getCurrentPosition()+robot.backLeft.getCurrentPosition())/4-pastWheelAverage);
        pastWheelAverage = deltaDistance;
        deltaDistance = 4*deltaDistance/PULSES_PER_REVOLUTION;
        double inchesPerSecond = deltaDistance/deltaTime;

        //SPEED PI
        double speedError = SPEED - inchesPerSecond;
        speedIntegral += (speedError * deltaTime);
        double speedOutput = (P * speedError + I * speedIntegral);
        //

            //ANGLE PI
        /*    angleError += Math.sin(DIRECTION)*deltaDistance;
            angleIntegral += (angleError * deltaTime);
            double angleOutput = (1/90 * angleError);// + (1 * angleIntegral);*/
            //

            robot.frontLeft.setPower(speedOutput);//+angleOutput);
            robot.frontRight.setPower(speedOutput);//-angleOutput);
            robot.backLeft.setPower(speedOutput);//+angleOutput);
            robot.backRight.setPower(speedOutput);//-angleOutput);

            deltaDisplacement += Math.cos(DIRECTION)*deltaDistance;
            telemetry.addData("speedError:      ", speedError);
            telemetry.addData("speedIntegral:   ", speedIntegral);
            telemetry.addData("speedOutput:     ", speedOutput);
            //telemetry.addData("angleError:      ", angleError);
            //telemetry.addData("angleIntegral:   ", angleIntegral);
            //telemetry.addData("angleOutput:     ", angleOutput);
            telemetry.addData("inchesPerSecond: ", inchesPerSecond);
    }
}
