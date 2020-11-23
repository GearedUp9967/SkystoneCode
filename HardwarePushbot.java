package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwarePushbot {
    DcMotor left;
    DcMotor right;

    HardwarePushbot() {

    }

    public void init(HardwareMap hw)
        {
        left=hw.dcMotor.get("left");
        right=hw.dcMotor.get("right");

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
