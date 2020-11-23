/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * {@link Guyrow} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "PID tuner", group = "Sensor")

public class Guyrow extends LinearOpMode {

    HardwareMike robot=new HardwareMike();

    Button b=new Button();
    Button a=new Button();

    int number=0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Wait until we're told to go
        waitForStart();


        PID pid=new PID(0,0,0);

        double power=0;
        boolean paused=true;
        pid.restart();
        pid.setTarget(90);

        while(opModeIsActive()) {
            b.update(gamepad1.b);
            a.update(gamepad1.a);


            String term="";

            if(number==0)
            {
                term="P";
                pid.P= Math.max(0,pid.P+gamepad1.left_stick_y*-0.001);
            }
            else if(number==1)
            {
                term="I";
                pid.I= Math.max(0,pid.I+gamepad1.left_stick_y*-0.001);
            }
            else
            {
                term="D";
                pid.D= Math.max(0,pid.D+gamepad1.left_stick_y*-0.001);
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


            if(!paused) {

                power = pid.update(angle);
                robot.forward(0, -power);
            }


            telemetry.addData("Paused",paused);
            telemetry.addData("P",pid.P);
            telemetry.addData("I",pid.I);
            telemetry.addData("D",pid.D);
            telemetry.addData("Angle",angle);
            telemetry.addData("Term",term);
            telemetry.update();

        }


    }

    public void turnTo(double target) {

        double angle;
        while(opModeIsActive())
        {
            angle=robot.getAngle();
            telemetry.addData("Pos",angle);

            telemetry.update();


            if(Math.abs(angle-target)>5) {
                double power = 0.1;

                if (angle - target < 0) {
                    power = -power;
                }

                robot.forward(0, -power);
            }
            else
            {
                robot.stop();
                break;
            }

        }

        sleep(500);

        while(opModeIsActive())
        {
            angle=robot.getAngle();
            telemetry.addData("Pos",angle);

            telemetry.update();


            if(Math.abs(angle-target)>1) {
                double power = 0.05;

                if (angle - target < 0) {
                    power = -power;
                }

                robot.forward(0, -power);
            }
            else
            {
                robot.stop();
                break;
            }

        }
    }




}
