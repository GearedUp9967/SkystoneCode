package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="child's play")

public class MsCarnival extends OpMode {
    HardwareMsCarnival robot = new HardwareMsCarnival();
    Button a=new Button();

    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        a.update(gamepad1.a);

        if (a.wasPressed()) {
            robot.toggleGrabber();
        }
        if (gamepad1.dpad_up) {
            robot.powerForward(1);
        } else if (gamepad1.dpad_down) {
            robot.powerForward(-1);
        } else {
            robot.powerForward(0);
        }
        if (gamepad1.dpad_left) {
            robot.powerLeft(1);
        } else if (gamepad1.dpad_right) {
            robot.powerLeft(-1);
        } else {
            robot.powerLeft(0);
        }
        robot.slide.setPower(gamepad1.right_stick_y);
    }
}