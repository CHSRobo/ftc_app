package org.firstinspires.ftc.team11750;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Bellatorum: Teleop Mecha", group="Bellatorum")

public class BellatorumTeleop19 extends OpMode {

    //https://www.google.com/url?sa=i&source=images&cd=&ved=2ahUKEwjEgsu8-NjkAhWkTt8KHcSJAX4QjRx6BAgBEAQ&url=https%3A%2F%2Ffunnyjunk.com%2Fchannel%2Fartwork%2FLip%2Beyes%2Bnose%2Bsketch%2Btechnique%2FMepvLZr%2F12&psig=AOvVaw0db2BgQgTG40Xyf6fzJSZi&ust=1568846877899616

    private BellatorumHardware19 robot = new BellatorumHardware19();

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        double maxSpeed = .69; //Defines what fraction of speed the robot will run at
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //this literally does pythagorean therom. idk why, but it does
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //calculates heading

        if (gamepad1.right_bumper) {
            /*robotAngle = Math.PI * 7 / 4;
            r = 1;*/
            robot.frontRight.setPower(maxSpeed);
            robot.frontLeft.setPower(maxSpeed);
            robot.backRight.setPower(-maxSpeed);
            robot.backLeft.setPower(-maxSpeed);
        } else if (gamepad1.left_bumper) {
            /*robotAngle = Math.PI * 3 / 4;
            r = 1;*/
            robot.frontRight.setPower(-maxSpeed);
            robot.frontLeft.setPower(-maxSpeed);
            robot.backRight.setPower(maxSpeed);
            robot.backLeft.setPower(maxSpeed);
        }

        double rightX = gamepad1.right_stick_x;
        double leftX = gamepad1.left_stick_x;
        final double v1 = (r * Math.sqrt(2) * Math.cos(robotAngle) + rightX );
        final double v2 = (r * Math.sqrt(2) * Math.sin(robotAngle) - rightX );
        final double v3 = (r * Math.sqrt(2) * Math.sin(robotAngle) + rightX );
        final double v4 = (r * Math.sqrt(2) * Math.cos(robotAngle) - rightX );

        robot.frontLeft.setPower(-v1 * maxSpeed);
        robot.frontRight.setPower(-v2 * maxSpeed);
        robot.backLeft.setPower(-v3 * maxSpeed);
        robot.backRight.setPower(-v4 * maxSpeed);

    }
}