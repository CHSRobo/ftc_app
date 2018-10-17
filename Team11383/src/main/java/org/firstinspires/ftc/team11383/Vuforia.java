package org.firstinspires.ftc.team11383;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Walt on 02/05/18.
 */
@Autonomous(name = "Vuforia")
public class Vuforia extends LinearOpMode
{
    VuforiaLocalizer vudforiaLocalizer;
    VuforiaLocalizer v;
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.update();
            idle();
        }
    }
}
