package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="RedAuto")
public class RedAuto extends BaseAutonomousIntake {

    StoneVision vision = new StoneVision(true);
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
        pidTurnDirectTest(90,0.017);
        sleep(100);
        turnDirect(90,0.1,0.5);
        sleep(100);
        robot.openRight();
        robot.closeCap();

        if(pos== StoneVision.Position.RIGHT)
        {
            encoderDrive(-1,-5,100);
            encoderStrafeRight(0.4,22,1000);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,9,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-79,1000,90);
            sleep(100);
            encoderStrafeRight(.5,10,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,10,100);
            sleep(100);
            robot.resetRight();
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(1,94,100,90);
            robot.openRight();
            encoderDrive(.4,10,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderStrafeRight(.5,11,100);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,12,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-92,1000,90);
            sleep(100);
            encoderStrafeRight(.5,12,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,5,100);
            pidTurn(180,0.01,false);
            encoderDrive(-0.5,-6,100);
            robot.closePuller();
            sleep(1000);
            encoderDrive(1,8,100);
            robot.rearleft.setPower(.15);
            robot.rearright.setPower(1);
            robot.frontleft.setPower(.15);
            robot.frontright.setPower(1);
            sleep(1500);
            robot.openPuller();
            sleep(100);
            encoderDrivePID(1,20,1000,90);
        }
        else if(pos== StoneVision.Position.MIDDLE)
        {
            encoderDrive(1,4,100);
            encoderStrafeRight(0.4,22,1000);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,9,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-84,1000,90);
            sleep(100);
            encoderStrafeRight(.5,11,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,11,100);
            sleep(100);
            robot.resetRight();
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(1,100,100,90);
            robot.openRight();
            encoderDrive(.4,10,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderStrafeRight(.5,9,100);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,11,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-102,1000,90);
            sleep(100);
            encoderStrafeRight(.5,13,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,5,100);
            pidTurn(180,0.01,false);
            encoderDrive(-0.5,-6,100);
            robot.closePuller();
            sleep(1000);
            encoderDrive(1,8,100);
            robot.rearleft.setPower(.15);
            robot.rearright.setPower(1);
            robot.frontleft.setPower(.15);
            robot.frontright.setPower(1);
            sleep(1500);
            robot.openPuller();
            sleep(100);
            encoderDrivePID(1,20,1000,90);
            /*encoderDrive(.8,40,100);
            robot.openPuller();
            sleep(200);
            encoderStrafeRight(1,40,100);*/
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderDrive(1,11,1000);
            encoderStrafeRight(0.4,22,1000);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,9,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            //encoderDrive(-1,-86,1000);
            encoderDrivePID(-1,-93,1000,90);
            sleep(100);
            encoderStrafeRight(.5,10,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,10,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            robot.resetRight();
            encoderDrivePID(.9,114,100,90);
            robot.openRight();
            encoderDrive(.4,10,100);
            sleep(100);
            encoderStrafeRight(.5,10,100);
            robot.closeRight();
            sleep(500);
            robot.liftRight();
            sleep(200);
            encoderStrafeLeft(.5,11,100);
            sleep(100);
            turnDirect(90,0.1,0.5);
            sleep(200);
            encoderDrivePID(-1,-110,1000,90);
            sleep(100);
            encoderStrafeRight(.5,12,100);
            robot.dropRight();
            sleep(200);
            encoderStrafeLeft(.5,5,100);
            pidTurn(180,0.01,false);
            encoderDrive(-0.5,-6,100);
            robot.closePuller();
            sleep(1000);
            encoderDrive(1,8,100);
            robot.rearleft.setPower(.15);
            robot.rearright.setPower(1);
            robot.frontleft.setPower(.15);
            robot.frontright.setPower(1);
            sleep(1500);
            robot.openPuller();
            sleep(100);
            encoderDrivePID(1,20,1000,90);


        }


    }
}
