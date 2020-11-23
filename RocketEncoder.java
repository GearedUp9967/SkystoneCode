package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Δv")
public class RocketEncoder extends LinearOpMode {

    HardwareMike robot = new HardwareMike();

    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();


        int inches = 50;
        int rltarget;
        int rrtarget;
        int fltarget;
        int frtarget;

        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rltarget = (int) (robot.rearleft.getCurrentPosition() + 80*COUNTS_PER_INCH);
            rrtarget = (int) (robot.rearright.getCurrentPosition() + 80*COUNTS_PER_INCH);
            fltarget = (int) (robot.frontleft.getCurrentPosition() + 80*COUNTS_PER_INCH);
            frtarget = (int) (robot.frontright.getCurrentPosition() + 80*COUNTS_PER_INCH);


            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            double acceleration = 0.0006;
            double speed = 0;
            double time = 0;



            while (opModeIsActive() &&
                    robot.rearleft.getCurrentPosition() < rltarget / 2.0 &&
                    robot.rearright.getCurrentPosition() < rrtarget / 2.0 &&
                    robot.frontleft.getCurrentPosition() < fltarget / 2.0 &&
                    robot.frontright.getCurrentPosition() < frtarget / 2.0) {

                time = runtime.milliseconds();
                speed = time * acceleration;

                robot.rearleft.setPower(Math.abs(speed));
                robot.rearright.setPower(Math.abs(speed));
                robot.frontleft.setPower(Math.abs(speed));
                robot.frontright.setPower(Math.abs(speed));
            }

            double startspeed=speed;
            runtime.reset();
            while (opModeIsActive() &&
                    robot.rearleft.getCurrentPosition() < rltarget &&
                    robot.rearright.getCurrentPosition() < rrtarget &&
                    robot.frontleft.getCurrentPosition() < fltarget &&
                    robot.frontright.getCurrentPosition() < frtarget) {

                time = runtime.milliseconds();
                speed = startspeed-time * acceleration;

                robot.rearleft.setPower(Math.min(Math.abs(speed),0.2));
                robot.rearright.setPower(Math.min(Math.abs(speed),0.2));
                robot.frontleft.setPower(Math.min(Math.abs(speed),0.2));
                robot.frontright.setPower(Math.min(Math.abs(speed),0.2));
            }

            robot.rearleft.setPower(0);
            robot.rearright.setPower(0);
            robot.frontleft.setPower(0);
            robot.frontright.setPower(0);
            while(opModeIsActive())
            {
                telemetry.addData("FL",robot.frontleft.getCurrentPosition()-fltarget);
                telemetry.addData("FR",robot.frontright.getCurrentPosition()-frtarget);
                telemetry.addData("RL",robot.rearleft.getCurrentPosition()-rltarget);
                telemetry.addData("RR",robot.rearright.getCurrentPosition()-rrtarget);

            }
        }

    }
}
