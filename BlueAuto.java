package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="BlueAuto")
public class BlueAuto extends BaseAutonomousIntake {

    StoneVision vision = new StoneVision(false);
    StoneVision.Position pos = StoneVision.Position.UNKNOWN;
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);
        //vision.enable();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);

        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        while(!isStarted())
        {
            telemetry.addData("Pos: ", String.valueOf(vision.getPos()));
            telemetry.addData("pixels", vision.numPixels);
            telemetry.update();
        }
        pos = vision.getPos();


        /*
        robot.openRight();
        sleep(1000);
        robot.closeRight();
        sleep(1000);
        robot.liftRight();
        sleep(1000);
        robot.dropRight();
        sleep(1000000);
        */


        encoderDrive(.5,10,1000);
        sleep(100);
        pidTurnDirectTest(-90,0.017);
        sleep(100);
        turnDirect(-90,0.1,0.5);
        sleep(100);
        robot.openLeft();
        robot.closeCap();


        //Dont be gay be straight

        if(pos== StoneVision.Position.RIGHT)
        {
            encoderDrive(.5,3,100);
            sleep(100);
            encoderStrafeLeft(0.4,21,1000);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,11,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-95,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,10,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,9,100);
            sleep(100);
            robot.resetLeft();
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(1,113,100,-90);
            robot.openLeft();
            encoderDrive(.4,10,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderStrafeLeft(.4,12,100);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,12,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-110,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,12,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,5,100);
            pidTurn(-180,0.01,true);
            encoderDrive(-0.5,-7,100);

            robot.closePuller();
            sleep(1000);
            encoderDrive(1,10,100);
            robot.rearleft.setPower(1);
            robot.rearright.setPower(.15);
            robot.frontleft.setPower(1);
            robot.frontright.setPower(.15);
            sleep(1500);
            robot.openPuller();
            sleep(10);
            encoderDrivePID(1,20,1000,90);
        }
        else if(pos== StoneVision.Position.MIDDLE)
        {
            encoderDrive(-1,-6,100);
            sleep(100);
            encoderStrafeLeft(0.4,21,1000);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,10,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-89,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,9,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,8,100);
            sleep(100);
            robot.resetLeft();
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(1,102,100,-90);
            robot.openLeft();
            encoderDrive(.4,11,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderStrafeLeft(.4,12,100);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,12,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-99,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,12,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,5,100);
            pidTurn(-180,0.01,true);
            encoderDrive(-0.5,-7,100);

            robot.closePuller();
            sleep(1000);
            encoderDrive(1,10,100);
            robot.rearleft.setPower(1);
            robot.rearright.setPower(.15);
            robot.frontleft.setPower(1);
            robot.frontright.setPower(.15);
            sleep(1500);
            robot.openPuller();
            sleep(100);
            encoderDrivePID(1,20,1000,90);
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderDrive(-1,-13,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(100);
            encoderStrafeLeft(0.4,21,1000);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,10,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-79,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,9,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,8,100);
            sleep(100);
            robot.resetLeft();
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(1,94,100,-90);
            robot.openLeft();
            encoderDrive(.4,10,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderStrafeLeft(.4,12,100);
            robot.closeLeft();
            sleep(500);
            robot.liftLeft();
            sleep(200);
            encoderStrafeRight(.5,12,100);
            sleep(100);
            turnDirect(-90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-87,1000,-90);
            sleep(100);
            encoderStrafeLeft(.5,12,100);
            robot.dropLeft();
            sleep(200);
            encoderStrafeRight(.5,5,100);
            pidTurn(-180,0.01,true);
            encoderDrive(-0.5,-7,100);

            robot.closePuller();
            sleep(1000);
            encoderDrive(1,10,100);
            robot.rearleft.setPower(1);
            robot.rearright.setPower(.15);
            robot.frontleft.setPower(1);
            robot.frontright.setPower(.15);
            sleep(1500);
            robot.openPuller();
            sleep(100);
            encoderDrivePID(1,20,1000,90);
        }


    }
}
