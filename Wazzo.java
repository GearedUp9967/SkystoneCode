package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Wazzo")

public class Wazzo extends OpMode {
    HardwareWazzo robot = new HardwareWazzo();

    boolean FIELD_CENTRIC=false;

    Button x1=new Button();
    Button a2=new Button();
    Button y2=new Button();
    Button left2=new Button(5);
    Button right2=new Button(5);
    Button down2=new Button();

    double speed=1;
    double measuredSpeed;
    double lastPos;
    double lastTime;
    double armStartPos;
    boolean safety=true;

    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_FOOT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*12;

    public void init() {

        robot.init(hardwareMap);
        robot.closeLeftclamp();
    }

    public void loop() {
        x1.update(gamepad1.x);
        a2.update(gamepad2.a);
        y2.update(gamepad2.y);

        left2.update(gamepad2.dpad_left);
        right2.update(gamepad2.dpad_right);

        down2.update(gamepad2.dpad_down);

        double newY;
        double newX;

        double turn = 0;

        if (gamepad1.left_bumper) {

            turn = -0.45*speed;
        }

        if (gamepad1.right_bumper) {
            turn = 0.45*speed;
        }


        if(gamepad2.right_stick_button && gamepad2.left_stick_button)
        {
            safety=false;
        }

        if(x1.isToggleOn())
        {
            speed=0.3;
        }
        else
        {
            speed=1;
        }

        if(a2.isToggleOn())
        {
            robot.closeGrabber();
        }
        else
        {
            robot.openGrabber();
        }
        if (y2.isToggleOn()) {
            robot.closePuller();
        }
        else
        {
            robot.openPuller();
        }

        if(left2.getToggleNumber()==0)
        {
            robot.resetLeft();
        }
        else if(left2.getToggleNumber()==1)
        {
            robot.openLeft();
        }
        else if(left2.getToggleNumber()==2)
        {
            robot.closeLeft();
        }
        else if(left2.getToggleNumber()==3)
        {
            robot.liftLeft();
        }
        else if(left2.getToggleNumber()==4)
        {
            robot.dropLeft();
        }


        if(right2.getToggleNumber()==0)
        {
            robot.resetRight();
        }
        else if(right2.getToggleNumber()==1)
        {
            robot.openRight();
        }
        else if(right2.getToggleNumber()==2)
        {
            robot.closeRight();
        }
        else if(right2.getToggleNumber()==3)
        {
            robot.liftRight();
        }
        else if(right2.getToggleNumber()==4)
        {
            robot.dropRight();
        }




        if(down2.isToggleOn())
        {
            robot.openCap();
        }
        else
        {
            robot.closeCap();
        }

        if(gamepad2.x)
        {
            robot.powerIntake(1);
        }
        else if(gamepad2.b)
        {
            robot.powerIntake(-.5);
        }
        else
        {
            robot.powerIntake(0);
        }




        double spoolpower=-gamepad2.left_stick_y;
        if(spoolpower<0 && robot.spool.getCurrentPosition()>=-500)
        {
            if(safety) {
                spoolpower = 0;
            }
        }

        robot.spool.setPower(spoolpower);


        if(gamepad2.right_bumper)
        {
            robot.arm.setPower(.75);
            robot.closeGrabber();
        }


        //Retracting
        else if(gamepad2.left_bumper && (robot.arm.getCurrentPosition()>10 || !safety))
        {
            robot.arm.setPower(-.75);
            robot.fullyCloseGrabber();
        }

        else
        {
            robot.arm.setPower(0);
        }

        telemetry.addData("Horiz Enc",robot.arm.getCurrentPosition());






        //double angle = Math.atan2(s_y, s_x) - (Math.PI/4);
        //double magnitude = Math.sqrt(s_x * s_x + s_y * s_y)/2;

        //newY = Math.sin(angle) * magnitude;
        //newX = Math.cos(angle) * magnitude;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;




        newY = rotateVectorY(x,y,-Math.PI/4);
        newX = rotateVectorX(x,y, -Math.PI/4);

        robot.frontright.setPower(speed*(newX) + turn);
        robot.rearleft.setPower(speed*(newX) - turn);

        robot.rearright.setPower(speed*(newY) + turn);
        robot.frontleft.setPower(speed*(newY) - turn);



        telemetry.addData("angle",robot.getAngle());
        telemetry.addData("Safety mode",safety);
        telemetry.addData("spool",robot.spool.getCurrentPosition());
        telemetry.update();


    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }
}
