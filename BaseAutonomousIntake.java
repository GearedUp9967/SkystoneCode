package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.ElementType;

public class BaseAutonomousIntake extends LinearOpMode {

    HardwareWazzo robot=new HardwareWazzo();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6;     //Yellow Jackets
    static final double     DRIVE_GEAR_REDUCTION    = 2;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);

    public void encoderDrive(double speed,
                             double inches,
                             double timeoutS) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active  |-----------------
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rearleft.setPower(Math.abs(speed));
            robot.frontleft.setPower(Math.abs(speed));
            robot.rearright.setPower(Math.abs(speed));
            robot.frontright.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.rearright.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void backWheelEncoderDrive(double speed,
                                      double inches,
                                      double timeoutS) {
        int flTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            flTarget = robot.frontleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.frontleft.setTargetPosition(flTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontleft.setPower(Math.abs(speed));
            robot.frontright.setPower(Math.abs(speed));

            robot.rearleft.setPower(0);
            robot.rearright.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.frontright.isBusy())) {
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }





    public void encoderStrafeRight(double speed,
                                   double inches,
                                   double timeoutS) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.right(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.frontleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafeRightPID(double speed,
                                      double inches,
                                      double timeoutS,double target) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            PID pid=new PID(0.05);
            pid.setTarget(target);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.right(speed,0);

            double power=0;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.rearleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {

                power=pid.update(robot.getAngle());
                robot.right(speed,-power);

            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }





    public void encoderStrafeLeft(double speed,
                                  double inches,
                                  double timeoutS) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.frontleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafeLeftPID(double speed,
                                     double inches,
                                     double timeoutS,double target) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            PID pid=new PID(0.05);
            pid.setTarget(target);
            double power=0;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.rearleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {

                power=pid.update(robot.getAngle());
                robot.left(speed,-power);


            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void turnTo(double target,boolean clockwise) {

        double angle=robot.getAngle();
        double lastAngle=angle;
        boolean passedTarget=false;

        while (opModeIsActive()) {
            lastAngle=angle;
            angle = robot.getAngle();


            //check if the target is between current angle and previous angle so we can deactivate single direction mode
            if((circleDist(lastAngle,target)<circleDist(lastAngle,angle))&&(circleDist(angle,target)<circleDist(angle,lastAngle)))
            {
                passedTarget=true;

            }



            //telemetry.addData("Pos", angle);
            //telemetry.addData("Passed Target",passedTarget);
            //telemetry.update();


            if (Math.abs(angle - target) > 1) {
                double power = 0.1;

                //drive in the specified direction, unless we pass the target. Then go towards the target the fastest way.
                if(!passedTarget) {
                    if(clockwise) {
                        robot.forward(0, power);
                    }
                    else
                    {
                        robot.forward(0,-power);
                    }
                }
                else
                {
                    if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
                    {
                        robot.forward(0, power);
                    }
                    else
                    {
                        robot.forward(0, -power);
                    }
                }
            } else {
                robot.stop();
                break;
            }


        }

    }



    //this function turns to the target in the shortest direction.
    public void turnToDirect(double target) {

        double angle;

        while (opModeIsActive()) {
            angle = robot.getAngle();

            //telemetry.addData("Pos", angle);

            //telemetry.update();


            if (Math.abs(angle - target) > 1) {
                double power = 0.1;

                //Go towards the target the fastest way.

                if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
                {
                    robot.forward(0, power);
                }
                else {
                    robot.forward(0, -power);
                }

            } else {
                robot.stop();
                break;
            }
        }
    }

    private double clockwiseDist(double angle1,double angle2)
    {
        if(angle1>=angle2)
        {
            return angle1-angle2;
        }
        else
        {
            return 360-Math.abs(angle2-angle1);
        }
    }

    private double counterClockwiseDist(double angle1,double angle2)
    {
        if(angle2>=angle1)
        {
            return angle2-angle1;
        }
        else
        {
            return 360-Math.abs(angle1-angle2);
        }
    }



    private double circleDist(double angle1,double angle2)
    {
        return Math.min(clockwiseDist(angle1,angle2),counterClockwiseDist(angle1,angle2));
    }

    public void encoderDrivePID(double speed,
                                double inches,
                                double timeoutS,double target) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rlTarget = robot.rearleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            flTarget = robot.frontleft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rrTarget = robot.rearright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            frTarget = robot.frontright.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.rearleft.setTargetPosition(rlTarget);
            robot.frontleft.setTargetPosition(flTarget);
            robot.rearright.setTargetPosition(rrTarget);
            robot.frontright.setTargetPosition(frTarget);

            // Turn On RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //robot.forward(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            PID pid=new PID(0.12);
            pid.setTarget(target);

            double power=0;
            ElapsedTime timer=new ElapsedTime();
            timer.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.rearleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {

                power=pid.update(robot.getAngle());


                robot.forward(speed/*Math.min(1,timer.milliseconds()/1000)*/,-power/*Math.min(1,timer.milliseconds()/1000)*/);
                //telemetry.addData("Angle",robot.getAngle());
                //telemetry.update();


            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }





    public void turnDirect(double target, double speed, double epsilon) {


        double power=speed;

        double angle;

        while (opModeIsActive()) {

            angle = robot.getAngle();
            if (Math.abs(target-angle)<epsilon)
            {
                break;

            }



            //telemetry.addData("Pos", angle);
            //telemetry.update();

            //Go towards the target the fastest way.

            if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
            {
                robot.forward(0, power);
            }
            else {
                robot.forward(0, -power);
            }

        }

        robot.stop();
    }

    public void pidTurnDirect(double target, double p) {


        PID pid=new PID(p,0,0);
        pid.restart();
        pid.setTarget(target);
        double power=0;

        double angle;
        ElapsedTime timer=new ElapsedTime();
        boolean in=false;
        while (opModeIsActive()) {

            angle = robot.getAngle();
            if (Math.abs(target-angle)<.5)
            {
                break;

            }



            //telemetry.addData("Pos", angle);
            //telemetry.update();

            power = Math.max(0.1,Math.abs(pid.update(angle)));

                //Go towards the target the fastest way.

                if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
                {
                    robot.forward(0, power);
                }
                else {
                    robot.forward(0, -power);
                }

            }

            robot.stop();
        }



    public void pidTurn(double target, double p,boolean clockwise) {



        double angle=robot.getAngle();
        double lastAngle=angle;
        boolean passedTarget=false;

        while (opModeIsActive()) {
            lastAngle=angle;
            angle = robot.getAngle();


            //check if the target is between current angle and previous angle so we can deactivate single direction mode
            // 12/11/19 changed from < to <= so that if the lastAngle==target or angle==target it will still work
            if((circleDist(lastAngle,target)<=circleDist(lastAngle,angle))&&(circleDist(angle,target)<=circleDist(angle,lastAngle)))
            {
                passedTarget=true;

            }



            //telemetry.addData("Pos", angle);
            //telemetry.addData("Passed Target",passedTarget);
            //telemetry.update();


            if (Math.abs(angle - target) > 2) {
                double power = Math.max(0.1,p*circleDist(angle,target));

                //drive in the specified direction, unless we pass the target. Then go towards the target the fastest way.
                if(!passedTarget) {
                    if(clockwise) {
                        robot.forward(0, power);
                    }
                    else
                    {
                        robot.forward(0,-power);
                    }
                }
                else
                {
                    if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
                    {
                        robot.forward(0, power);
                    }
                    else
                    {
                        robot.forward(0, -power);
                    }
                }
            } else {
                robot.stop();
                break;
            }


        }

    }

    public void pidTurnDirectTest(double target, double p) {


        PID pid=new PID(p,0,0);
        pid.restart();
        pid.setTarget(target);
        double power=0;

        double angle;
        ElapsedTime timer=new ElapsedTime();
        boolean in=false;
        while (opModeIsActive()) {

            angle = robot.getAngle();
            if (Math.abs(target-angle)<.5)
            {
                break;

            }



            //telemetry.addData("Pos", angle);
            //telemetry.update();

            power = Math.max(0,Math.abs(pid.update(angle)));

            //Go towards the target the fastest way.

            if(clockwiseDist(angle,target)<counterClockwiseDist(angle,target))
            {
                robot.forward(0, power);
            }
            else {
                robot.forward(0, -power);
            }

        }

        robot.stop();
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}