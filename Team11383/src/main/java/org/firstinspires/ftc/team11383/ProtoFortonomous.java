// ProtoFortonomous.java
// Prototype Autonomous currently in use
// Focus on development of basic features
// Fortonomous.java will be used for more complex future activities

package org.firstinspires.ftc.team11383;

/**
 * Created by robotics on 10/20/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="ProtoFortonomous")
public class ProtoFortonomous extends LinearOpMode{
    private HardwareFortissimus2 robot = new HardwareFortissimus2();



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        robot.leadScrew.setPower(1);
        sleep (12000);
        robot.leadScrew.setPower(0);
        //sleep(1000);

        //robot.hook.setPosition(0);

        robot.leftFrontMotor.setPower(-1);
        robot.rightFrontMotor.setPower(1);
        robot.leftBackMotor.setPower(-1);
        robot.rightBackMotor.setPower(1);
        sleep(750);

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        
    }
}
