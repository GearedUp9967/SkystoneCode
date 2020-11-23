package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="PlateGrab")

public class PlateGrab extends BaseAutonomousIntake {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.closePuller();
        /*
        robot.closePuller();
        sleep(1000);
        encoderDrive(1,5,100);
        robot.rearleft.setPower(.15);
        robot.rearright.setPower(1);
        robot.frontleft.setPower(.15);
        robot.frontright.setPower(1);
        sleep(1800);
        robot.openPuller();
        sleep(100);*/
        encoderDrivePID(1,20,1000,90);

        while(opModeIsActive())
        {
        }
        robot.openPuller();
        sleep(500);
    }
}
