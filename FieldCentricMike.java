package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Field Centric Mike")

public class FieldCentricMike extends OpMode {
    HardwareMike robot = new HardwareMike();

    boolean FIELD_CENTRIC=true;

    Button a2=new Button();
    Button b2=new Button();
    Button a1=new Button();
    double speed=1;

    public void init() {
        robot.init(hardwareMap);
    }

    public void loop() {
        a2.update(gamepad2.a);
        b2.update(gamepad2.b);
        a1.update(gamepad1.a);
        double newY;
        double newX;

        double turn = 0;

        if (gamepad1.left_bumper) {
            turn = -1;
        }

        if (gamepad1.right_bumper) {
            turn = 1;
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

        if(a1.getToggleMode()==true)
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


        robot.armLift.setPower(-gamepad2.left_stick_y*.4);




        double adjust=0;

        if(FIELD_CENTRIC)
        {
            adjust=robot.getAngle()*Math.PI/180;
        }


        newY = rotateVectorY(x,y,-Math.PI/4+adjust);
        newX = rotateVectorX(x,y, -Math.PI/4+adjust);


        robot.frontright.setPower(speed*newX + turn);
        robot.rearleft.setPower(speed*newX - turn);

        robot.rearright.setPower(speed*newY + turn);
        robot.frontleft.setPower(speed*newY - turn);
        telemetry.addData("angle",robot.getAngle());



    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }
}
