package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Gud Turning")
public class NonDiscontinuousTurning extends BaseAutonomousIntake {

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        waitForStart();
        double target=90;
        while (true)
        {

            pidTurnDirect(target,StraightStrafingPIDConstants.P);
            sleep(1000);
            target=target+90;
            if(target>180)
            {
                target=-90;
            }
        }
    }
}
