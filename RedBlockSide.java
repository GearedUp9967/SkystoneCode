package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


//@Autonomous(name="RedBlockSide")

public class RedBlockSide extends BaseAutonomous {
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
        //vision.disable();

        if(pos== StoneVision.Position.RIGHT)
        {

            encoderStrafeLeftPID(.5,27,100,0);
            robot.openGrips();
            encoderDrive(-.5,-5,100);
            encoderDrive(.7, 27, 100);
            encoderDrive(.4,5,100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1, -11, 100);
            sleep(200);
            encoderStrafeRightPID(.5,80,2000,0);
            sleep(100);
            encoderDrive(.8,10,100);
            robot.openGrabber();
            sleep(300);
            encoderDrivePID(-.8,-10,100,0);
            sleep(100);
            encoderStrafeLeftPID(.5,48,1000,0);
            encoderDrive(.4,14,100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1,-14,100);
            sleep(200);
            encoderStrafeRightPID(.5,82,2000,0);
            sleep(200);
            robot.openGrips();
            robot.armLift.setPower(.4);
            sleep(350);
            encoderDrive(.4, 18, 100);

            robot.armLift.setPower(0);
            robot.closeGrips();
            sleep(1000);
            encoderDrive(-.7,-25,2000);
            robot.forward(0,0.75);
            sleep(1500);
            encoderDrive(1,35,100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1,-35,1000);

        }

        else if(pos== StoneVision.Position.MIDDLE)
        {
            encoderStrafeLeftPID(.5,34,100,0);
            robot.openGrips();
            encoderDrive(-.4,-5,100);
            sleep(200);
            encoderDrive(.7, 27, 100);
            encoderDrive(.4,5,100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1, -11, 100);
            sleep(200);
            encoderStrafeRightPID(.5,90,2000,0);
            sleep(100);
            encoderDrive(.8,10,100);
            robot.openGrabber();
            sleep(300);
            encoderDrive(-.8,-10,100);
            sleep(100);
            encoderStrafeLeftPID(.5,56,1000,0);
            encoderDrive(.4,14,100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1,-14,100);
            sleep(200);
            encoderStrafeRightPID(.5, 90, 2000,0);
            sleep(200);
            robot.armLift.setPower(.4);
            sleep(350);
            encoderDrive(.4, 18, 100);

            robot.armLift.setPower(0);

            robot.closeGrips();
            sleep(1000);
            encoderDrive(-.7,-25,2000);
            robot.forward(0,0.75);
            sleep(1500);
            encoderDrive(1,35,100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1,-35,1000);
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderStrafeLeftPID(.3,18,100,0);
            robot.openGrips();
            encoderDrive(-.4,-8,100);
            sleep(200);
            encoderDrive(0.7, 27, 100);
            encoderDrive(.4,5,100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-1, -11, 100);
            sleep(200);
            encoderStrafeRightPID(.5, 106, 2000,0);
            sleep(200);
            robot.armLift.setPower(.4);
            sleep(350);
            encoderDrive(.4, 18, 100);

            robot.armLift.setPower(0);

            robot.closeGrips();
            sleep(1000);
            encoderDrive(-.7,-25,2000);
            robot.forward(0,0.75);
            sleep(1500);
            encoderDrive(1,35,100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1,-35,1000);
        }


    }
}
