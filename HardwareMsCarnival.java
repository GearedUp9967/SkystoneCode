package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMsCarnival {
    DcMotor left;
    DcMotor right;
    DcMotor slide;
    Servo grabber;

    public double GRABBER_OPEN=0.5;
    public double GRABBER_CLOSED=0.9;

    public boolean grabberOpen=true;

    public void init(HardwareMap hw) {
        left = hw.dcMotor.get("left");
        right = hw.dcMotor.get("right");
        slide = hw.dcMotor.get("slide");
        grabber = hw.servo.get("grabber");

        left.setDirection(DcMotor.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        openGrabber();

    }
    public void powerLeft(double power){
        right.setPower(-power);
        left.setPower(power);
    }
    public void powerForward(double power){
        right.setPower(-power);
        left.setPower(-power);
    }
    public void openGrabber() {
        grabber.setPosition(GRABBER_OPEN);
        grabberOpen=!grabberOpen;
    }

    public void closeGrabber() {
        grabber.setPosition(GRABBER_CLOSED);
        grabberOpen=!grabberOpen;
    }
/*    public void openGrabber() {
        grabber.setPosition(GRABBER_OPEN);
    }

    public void closeGrabber() {
        grabber.setPosition(GRABBER_CLOSED);
    }*/
    public void toggleGrabber() {
        if(grabberOpen) {
            closeGrabber();
        }
        else {
            openGrabber();
        }
    }
    public void powerSlide(double power){
        slide.setPower(power);
    }
}


