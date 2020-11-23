package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Straight Strafing")

public class TestingStraightStrafing extends LinearOpMode {

    HardwareMike robot=new HardwareMike();

    Button b=new Button();
    Button a=new Button();
    Button x=new Button();
    Button up=new Button();
    Button down=new Button();
    int number=0;
    boolean right=true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Wait until we're told to go
        waitForStart();


        PID pid=new PID(0,0,0);

        double power=0;
        boolean paused=true;
        pid.restart();
        pid.setTarget(0);

        while(opModeIsActive()) {
            b.update(gamepad1.b);
            a.update(gamepad1.a);
            x.update(gamepad1.x);
            up.update(gamepad1.dpad_up);
            down.update(gamepad1.dpad_down);
            String term="";


            if(number==0)
            {
                term="P";
                pid.P= Math.max(0,pid.P+gamepad1.left_stick_y*-0.001);
                if(up.wasPressed())
                {
                    pid.P= Math.max(0,pid.P+0.0001);
                }
                else if(down.wasPressed())
                {
                    pid.P= Math.max(0,pid.P-0.0001);
                }
            }
            else if(number==1)
            {
                term="I";
                pid.I= Math.max(0,pid.I+gamepad1.left_stick_y*-0.001);
                if(up.wasPressed())
                {
                    pid.I= Math.max(0,pid.P+0.0001);
                }
                else if(down.wasPressed())
                {
                    pid.I= Math.max(0,pid.P-0.0001);
                }
            }
            else
            {
                term="D";
                pid.D= Math.max(0,pid.D+gamepad1.left_stick_y*-0.001);
                if(up.wasPressed())
                {
                    pid.D= Math.max(0,pid.P+0.0001);
                }
                else if(down.wasPressed())
                {
                    pid.D= Math.max(0,pid.P-0.0001);
                }
            }
            double angle=robot.getAngle();


            if(b.wasPressed()){
                paused=!paused;
                if(!paused)
                {
                    pid.restart();
                }
            }

            if(a.wasPressed()){
                number=(number+1)%3;
            }

            if(x.wasPressed()){
                right=!right;
            }


            String direction="";
            if(!paused) {

                power = pid.update(angle);
                if(right) {
                    robot.right(.5, -power);

                }
                else {
                    robot.left(.5, -power);
                }
            }
            else
            {
                robot.stop();
            }


            if(right) {
                direction="right";
            }
            else {
                direction="left";
            }

            telemetry.addData("Paused",paused);
            telemetry.addData("P",pid.P);
            telemetry.addData("I",pid.I);
            telemetry.addData("D",pid.D);
            telemetry.addData("Angle",angle);
            telemetry.addData("Term",term);
            telemetry.addData("Direction",direction);
            telemetry.update();

        }


    }





}
