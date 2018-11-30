package org.firstinspires.ftc.team11383;

/**
 * Created by robotics on 10/20/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ProtoFortonomous")
public class ProtoFortonomous extends FortissimusAuto{
//    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void runOpMode() throws InterruptedException {

        // robot.leadScrew.setPower(1);
        // sleep (18000);
        // robot.leadScrew.setPower(0);
        // sleep(1000);


        robot.leftBackMotor.setPower(1);
        robot.rightFrontMotor.setPower(1);
        sleep(2500);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

        robot.relic.setPosition(1);
        sleep(500);
        robot.relic.setPosition(0);

        robot.rightFrontMotor.setPower(1);
        robot.rightBackMotor.setPower(1);
        robot.leftBackMotor.setPower(1);
        robot.leftFrontMotor.setPower(1);
        sleep(750);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);

        robot.leftBackMotor.setPower(.75);
        robot.rightFrontMotor.setPower(.75);
        sleep(5000);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

    }
}
