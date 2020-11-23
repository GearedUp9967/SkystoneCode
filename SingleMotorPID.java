package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name="Single motor test")
public class SingleMotorPID extends OpMode {

    DcMotorEx fl;
    DcMotor fr;
    @Override
    public void init() {
        fl=(DcMotorEx) hardwareMap.dcMotor.get("frontleft");
        fr=hardwareMap.dcMotor.get("frontright");
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        fl.setVelocity(90, AngleUnit.DEGREES);
        fr.setPower(0.1);
    }
}
