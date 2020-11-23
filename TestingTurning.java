package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="testing turning")

public class TestingTurning extends BaseAutonomous {

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        pidTurn(135,0.0110);

        robot.stop();
        while(opModeIsActive())
        {
            telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }

}
