package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseAutonomous extends LinearOpMode {

    HardwareMike robot=new HardwareMike();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6;     //Yellow Jackets
    static final double     DRIVE_GEAR_REDUCTION    = 2;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void encoderDrive(double speed,
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


            PID pid=new PID(0.03);
            pid.setTarget(target);

            // reset the timeout time and start motion.

            robot.right(speed,0);

            double power=0;
            runtime.reset();
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
            PID pid=new PID(0.03);
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


    public void turnTo(double target) {

        double angle;


        while (opModeIsActive()) {
            angle = robot.getAngle();

            double clockwiseDist=0;
            if(target>angle)
            {
                clockwiseDist=target-angle;

            }
            else
            {
                clockwiseDist=360-Math.abs(target-angle);
            }

            double counterClockwiseDist=0;

            if(target<angle)
            {
                counterClockwiseDist=angle-target;
            }
            else
            {
                counterClockwiseDist=360-Math.abs(target-angle);
            }


            telemetry.addData("Pos", angle);

            telemetry.update();


            if (Math.abs(angle - target) > 1) {
                double power = 0.1;

                if (counterClockwiseDist<clockwiseDist) {
                    power = -power;
                }

                robot.forward(0, -power);
            } else {
                robot.stop();
                break;
            }

        }

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
            robot.right(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            PID pid=new PID(0.04);
            pid.setTarget(target);

            double power=0;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.frontleft.isBusy() && robot.frontright.isBusy() && robot.rearright.isBusy())) {

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

    public void pidTurn(double target, double p) {
        PID pid=new PID(p,0,0);
        pid.restart();
        pid.setTarget(target);
        double power=0;
        ElapsedTime timer=new ElapsedTime();

        while(opModeIsActive()) {
            double angle=robot.getAngle();
            if (Math.abs(target-angle)<2)
            {
                break;

            }


            power = Math.max(0.1,pid.update(angle));
            robot.forward(0, -power);

            telemetry.addData("Angle",angle);
            telemetry.update();

        }
        robot.stop();

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
