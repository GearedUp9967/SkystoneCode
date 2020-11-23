package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {
    DcMotor encoder;
    private double COUNTS_PER_INCH=8192.0/(3.14159*4);
    @Override
    public void runOpMode() throws InterruptedException {
        encoder=hardwareMap.dcMotor.get("encoder");
        waitForStart();
        while(true) {
            telemetry.addData("Inches: ",encoder.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.update();
        }
    }
}
