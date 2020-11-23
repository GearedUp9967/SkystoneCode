/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NoobPushBot {
    DcMotor left;
    DcMotor right;
    DcMotor rise;
    Servo grab;
    Button a1 = new Button();

    HardwareMap hwMap;

    public NoobPushBot() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        left = hwMap.dcMotor.get("left");
        right = hwMap.dcMotor.get("right");
        rise = hwMap.dcMotor.get("rise");

        rise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop() {
        a1.update(gamepad1.a1);


    }
}
*/