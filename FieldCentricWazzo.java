package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FieldCentricWazzo")

public class FieldCentricWazzo extends OpMode {
    HardwareWazzo robot = new HardwareWazzo();

    boolean FIELD_CENTRIC=true;

    Button x1=new Button();
    Button a2=new Button();
    Button y2=new Button();

    double speed=1;
    double measuredSpeed;
    double lastPos;
    double lastTime;
    double armStartPos;


    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_FOOT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*12;

    public void init() {

        robot.init(hardwareMap);
    }

    public void loop() {
        x1.update(gamepad1.x);
        a2.update(gamepad2.a);
        y2.update(gamepad2.y);

        double newY;
        double newX;

        double turn = 0;

        if (gamepad1.left_bumper) {
            turn = -0.45;
        }

        if (gamepad1.right_bumper) {
            turn = 0.45;
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

       // if(y2.isToggleOn())
        {
            //robot.closeGrips();
        }
        //else
        {
            //robot.openGrips();
        }//

        if(gamepad2.x)
        {
            robot.powerIntake(1);
        }
        else if(gamepad2.b)
        {
            robot.powerIntake(-1);
        }
        else
        {
            robot.powerIntake(0);
        }


        robot.arm.setPower(-gamepad2.left_stick_y);



        //double angle = Math.atan2(s_y, s_x) - (Math.PI/4);
        //double magnitude = Math.sqrt(s_x * s_x + s_y * s_y)/2;

        //newY = Math.sin(angle) * magnitude;
        //newX = Math.cos(angle) * magnitude;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;


        robot.spool.setPower(-gamepad2.right_stick_y);




        double adjust=0;
        if(FIELD_CENTRIC)
        {
            adjust=robot.getAngle()*Math.PI/180;
        }

        newY = rotateVectorY(x,y,-Math.PI/4+adjust);
        newX = rotateVectorX(x,y, -Math.PI/4+adjust);


        double scale=1;

        double fr=speed*newX + turn;
        double rl=speed*newX - turn;
        double rr=speed*newY + turn;
        double fl=speed*newY - turn;


        // 12/12/2019: if one of the motor powers is greater than one, rescale all motor powers so that the max one equals 1.
        // This should help with turning while strafing.
        if(fr>=1 || rl>=1 || rr>=1 || fl>=1)
        {
            scale=Math.max(Math.max(fr,rl),Math.max(rr,fl));
        }

        robot.frontright.setPower(fr/scale);
        robot.rearleft.setPower(rl/scale);

        robot.rearright.setPower(rr/scale);
        robot.frontleft.setPower(fl/scale);
        telemetry.addData("angle",robot.getAngle());



    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }
}
