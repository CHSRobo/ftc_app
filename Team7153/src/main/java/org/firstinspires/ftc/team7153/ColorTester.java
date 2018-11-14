package org.firstinspires.ftc.team7153;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;
import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.team7153.HardwareByrd.PULSES_PER_REVOLUTION;

@TeleOp(name="ColorTester")
public class ColorTester extends OpMode{
     private HardwareByrd robot = new HardwareByrd();

    double[] red = {0,100};
    double[] green = {0,100};
    double[] blue = {0,100};
    double[] alpha = {0,100};
    int mode = 0;
    long runTime = 0;
    @Override
    public void init() {
    }	
    @Override	
    public void loop() {

        if(robot.sensorColor.alpha()>alpha[0] && robot.sensorColor.alpha()<alpha[1]) {
            telemetry.addData("Object ", "Detected" );
            if (robot.sensorColor.red() > red[0] && robot.sensorColor.red() < red[1] && robot.sensorColor.green() > green[0] && robot.sensorColor.green() < green[1] && robot.sensorColor.blue() > blue[0] && robot.sensorColor.blue() < blue[1]) {
                telemetry.addData("Color: ", "Yellow" );
            } else {
                telemetry.addData("Color: ", "Other/White" );
            }
        } else {
            telemetry.addData("Object ", "Undetected" );
        }

        if(gamepad1.a){
            mode = 0;
        } else if(gamepad1.b){
            mode = 1;
        } else if(gamepad1.x){
            mode = 2;
        } else if(gamepad1.y){
            mode = 3;
        }


        if(mode == 0){
            if(runTime+10<currentTimeMillis()){
                if(gamepad1.dpad_up){
                    alpha[0]++;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_down){
                    alpha[0]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_left){
                    alpha[1]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_right){
                    alpha[1]++;
                    runTime=currentTimeMillis();
                }
            }
        } else if(mode == 1){
            if(runTime+10<currentTimeMillis()){
                if(gamepad1.dpad_up){
                    red[0]++;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_down){
                    red[0]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_left){
                    red[1]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_right){
                    red[1]++;
                    runTime=currentTimeMillis();
                }
            }
        } else if(mode == 2){
            if(runTime+10<currentTimeMillis()){
                if(gamepad1.dpad_up){
                    green[0]++;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_down){
                    green[0]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_left){
                    green[1]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_right){
                    green[1]++;
                    runTime=currentTimeMillis();
                }
            }
        } else if(mode == 3){
            if(runTime+10<currentTimeMillis()){
                if(gamepad1.dpad_up){
                    blue[0]++;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_down){
                    blue[0]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_left){
                    blue[1]--;
                    runTime=currentTimeMillis();
                } else if(gamepad1.dpad_right){
                    blue[1]++;
                    runTime=currentTimeMillis();
                }
            }
        }

        telemetry.addData("R: ", robot.sensorColor.red() );
        telemetry.addData("G: ", robot.sensorColor.blue() );
        telemetry.addData("B: ", robot.sensorColor.green() );
        telemetry.addData("A: ", robot.sensorColor.alpha() );

        telemetry.update();
    }
}
