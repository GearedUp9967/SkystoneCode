package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Encoder PID Strafing")

public class EncoderPIDStrafing extends BaseAutonomous {


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Wait until we're told to go
        waitForStart();

        while(opModeIsActive()) {
            encoderStrafeRightPID(0.2, 20, 10, 0);
            encoderStrafeLeftPID(0.2, 20, 10, 0);
        }


    }





}
