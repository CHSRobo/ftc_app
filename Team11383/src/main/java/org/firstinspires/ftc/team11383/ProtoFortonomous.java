package org.firstinspires.ftc.team11383;

/**
 * Created by robotics on 10/20/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ProtoFortonomous")
public class ProtoFortonomous extends LinearOpMode{
    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.leadScrew.setPower(1);
        sleep (10000);
        robot.leadScrew.setPower(0);
        sleep(1000);
        
        robot.leftFrontMotor.setPower(1);
        robot.rightFrontMotor.setPower(-1);
        robot.leftBackMotor.setPower(-1);
        robot.rightBackMotor.setPower(1);
        sleep(750);
        
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        
        
    }
}
