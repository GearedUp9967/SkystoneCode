package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="pOoShBoTt")

public class Pushbot extends OpMode {
    HardwarePushbot robot=new HardwarePushbot();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.left.setPower(gamepad1.left_stick_y);
        robot.right.setPower(gamepad1.right_stick_y);
    }
}