package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Iterator;

@TeleOp(name="ServoSetter V2")

public class ServoSetter extends OpMode {

    private ArrayList<Servo> servos=new ArrayList<>();
    private Button rb=new Button();
    private Button lb=new Button();
    private Button up=new Button();
    private Button down=new Button();
    private int index=0;

    @Override
    public void init() {
        Iterator<Servo> servoIter;
        servoIter=hardwareMap.servo.iterator();
        while (servoIter.hasNext()) {
            servos.add(servoIter.next());
        }


    }

    @Override
    public void loop() {
        rb.update(gamepad1.right_bumper);
        lb.update(gamepad1.left_bumper);

        up.update(gamepad1.dpad_up);
        down.update(gamepad1.dpad_down);

        if(rb.wasPressed()) {
            index+=1;
            if(index>=servos.size()){
                index=0;
            }
        }
        else if(lb.wasPressed()) {
            index-=1;
            if(index<0){
                index=servos.size()-1;
            }
        }

        Servo servo=servos.get(index);

        double newPos=servo.getPosition();
        double increment;

        if(gamepad1.b)
        {
            increment=0.01;
        }
        else {
            increment=0.05;
        }

        if(up.wasPressed()) {
            newPos=servo.getPosition()+increment;
            if(newPos>1) {
                newPos=1;
            }
            servo.setPosition(newPos);
        }
        else if(down.wasPressed()) {
            newPos=servo.getPosition()-increment;
            if(newPos<0)
            {
                newPos=0;
            }
            servo.setPosition(newPos);

        }

        telemetry.addData("port: ",servo.getPortNumber());
        telemetry.addData("position: ",newPos);
        telemetry.update();
    }
}
