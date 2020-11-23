package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="distance sensor")
public class StrafeWithSensor extends LinearOpMode {


    public  ModernRoboticsI2cRangeSensor rangeSensor;
    HardwareSuperscobra robot=new HardwareSuperscobra();
    @Override public void runOpMode() {

        robot.init(hardwareMap);
        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // wait for the start button to be pressed
        waitForStart();

        sleep(1000);

        encoderStrafeLeftPIDRange(0.1,0,11);
        while (opModeIsActive()) {

            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.update();

        }
    }

    public void encoderStrafeLeftPIDRange(double speed, double targetAngle, double stopDist) {
        int rlTarget;
        int flTarget;
        int rrTarget;
        int frTarget;

        ElapsedTime runtime=new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left(speed,0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //PID pid=new PID(0.03);
            //pid.setTarget(targetAngle);
            double power=0;
            while (opModeIsActive() &&
                    getDist()>stopDist) {

                //power=pid.update(robot.getAngle());
                robot.left(speed,0);


            }

            // Stop all motion;
            robot.stop();

        }
    }

    public int getDist()
    {
        return rangeSensor.rawUltrasonic();
    }
}
