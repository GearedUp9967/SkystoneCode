package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MiNiMeCCeMiNiM")
public class Minimec extends OpMode {
    HardwareMinimec robot = new HardwareMinimec();

    public void init() {
        robot.init(hardwareMap);
    }

    public void loop() {


        double newY;
        double newX;

        double turn = 0;

        if (gamepad1.left_bumper) {
            turn = 0.45;
        }

        if (gamepad1.right_bumper) {
            turn = -0.45;
        }


        //double angle = Math.atan2(s_y, s_x) - (Math.PI/4);
        //double magnitude = Math.sqrt(s_x * s_x + s_y * s_y)/2;

        //newY = Math.sin(angle) * magnitude;
        //newX = Math.cos(angle) * magnitude;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;


        newY = rotateVectorY(x,y,-Math.PI/4);
        newX = rotateVectorX(x,y, -Math.PI/4);


        robot.frontright.setPower(newX + turn);
        robot.rearleft.setPower(newX - turn);

        robot.rearright.setPower(newY + turn);
        robot.frontleft.setPower(newY - turn);
        telemetry.addData("angle",robot.getAngle());

    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }

}
