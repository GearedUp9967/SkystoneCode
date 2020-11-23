package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="Mike")

public class Mike extends OpMode {
    HardwareMike robot = new HardwareMike();

    boolean FIELD_CENTRIC=false;

    Button a2=new Button();
    Button b2=new Button();
    Button x1=new Button();
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
        armStartPos=robot.armLift.getCurrentPosition();
    }

    public void loop() {
        a2.update(gamepad2.a);
        b2.update(gamepad2.b);
        x1.update(gamepad1.x);

        double newY;
        double newX;

        double turn = 0;

        if (gamepad1.left_bumper) {
            turn = -0.45;
        }

        if (gamepad1.right_bumper) {
            turn = 0.45;
        }

            if (gamepad1.right_trigger > 0) {
                robot.parker.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                robot.parker.setPower(-0.75);
            }
            else
            {
                robot.parker.setPower(0);
            }

        if(a2.wasPressed())
        {
            robot.toggleGrabber();
        }

        if(b2.getToggleMode()==false)
        {
              robot.closeGrips();
        }
        else
        {
            robot.openGrips();
        }

        if(x1.getToggleMode()==true)
        {
            speed=0.3;
        }
        else
        {
            speed=1;
        }
        //double angle = Math.atan2(s_y, s_x) - (Math.PI/4);
        //double magnitude = Math.sqrt(s_x * s_x + s_y * s_y)/2;

        //newY = Math.sin(angle) * magnitude;
        //newX = Math.cos(angle) * magnitude;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;


        robot.armLift.setPower(-gamepad2.left_stick_y*.71);




        double adjust=0;
        /*
        if(FIELD_CENTRIC)
        {
            adjust=robot.getAngle()*Math.PI/180;
        }
        */

        newY = rotateVectorY(x,y,-Math.PI/4+adjust);
        newX = rotateVectorX(x,y, -Math.PI/4+adjust);


        robot.frontright.setPower(speed*newX + turn);
        robot.rearleft.setPower(speed*newX - turn);

        robot.rearright.setPower(speed*newY + turn);
        robot.frontleft.setPower(speed*newY - turn);
        telemetry.addData("angle",robot.getAngle());
        telemetry.addData("Arm Encoder",robot.armLift.getCurrentPosition()-armStartPos);

        double pos=robot.rearleft.getCurrentPosition()/COUNTS_PER_FOOT;
        double time=getRuntime();
        measuredSpeed=(pos-lastPos)/(getRuntime()-lastTime);
        lastPos=pos;
        lastTime=time;
        telemetry.addData("Speed (ft/s)",measuredSpeed);


    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }
}
