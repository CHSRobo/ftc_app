package org.firstinspires.ftc.team11383;

/**
 * Created by robotics on 10/20/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Fortonomous")
public class Fortonomous extends FortissimusAuto{
    private HardwareFortissimus2 robot = new HardwareFortissimus2();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.leftBackMotor.setPower(1);
        robot.rightFrontMotor.setPower(1);
        sleep(1000);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

        robot.relic.setPosition(1);
    }
}
