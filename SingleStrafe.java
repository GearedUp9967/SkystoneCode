package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="single strafe")
public class SingleStrafe extends BaseAutonomousIntake {


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(!opModeIsActive()) {
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
        }

        pidTurnDirectTest(90,0.017);

        while(opModeIsActive()) {
            telemetry.addData("done Angle", robot.getAngle());
            telemetry.update();
        }
    }

}
