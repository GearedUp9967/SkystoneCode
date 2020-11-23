package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


//@Autonomous(name="BlueBlockSideIntake")
public class BlueBlockSideIntake extends BaseAutonomousIntake {
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
        //vision.disable();

        if(pos== StoneVision.Position.RIGHT)
        {
            encoderStrafeRight(.3,10,100);
            encoderDrive(-.5,-5,100);
            sleep(200);
            encoderDrive(0.7, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            sleep(100);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeLeftPID(.5, 99, 2000,0);
            sleep(200);
            //robot.openGrips();
            //robot.armLift.setPower(.35);
            //sleep(500);
            encoderDrive(.4, 15, 100);

            //while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            //sleep(500);
            encoderDrive(-1,-60,2000);
            //robot.openGrips();
            //robot.openGrabber();
            encoderStrafeRightPID(.5,43,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrivePID(.7,25,100,0);
            encoderStrafeRightPID(.5,15,100,0);
        }
        else if(pos== StoneVision.Position.MIDDLE)
        {
            encoderStrafeRightPID(.5,27,100,0);

            encoderDrive(.5,-5,100);
            encoderDrive(0.7, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            sleep(100);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeLeftPID(.5,83,1000,0);
            sleep(200);

            encoderDrive(.8,10,100);
            robot.powerIntake(-1);
            sleep(300);
            encoderDrive(-.8,-10,100);
            sleep(200);
            robot.powerIntake(0);
            encoderStrafeRightPID(.5,57,100,0);
            sleep(200);
            encoderDrive(.4,7,100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            sleep(100);
            robot.powerIntake(0);
            encoderDrive(-1,-17,100);
            sleep(200);
            encoderStrafeLeftPID(.5, 93, 2000,0);
            sleep(200);
//while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            //sleep(500);
            encoderDrive(.4, 15, 100);



            //robot.closeGrips();
            sleep(500);
            encoderDrive(-1,-60,2000);
            //robot.openGrips();
            //robot.openGrabber();
            encoderStrafeRightPID(.5,43,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrivePID(.7,25,100,0);
            encoderStrafeRightPID(.5,15,100,0);
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderStrafeRightPID(.4,18,100,0);
            //robot.openGrips();
            encoderDrive(-.2,-8,100);
            encoderDrive(0.7, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            //robot.closeGrabber();
            sleep(100);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeLeftPID(.5,75,1000,0);
            sleep(200);
            encoderDrive(.8,10,100);
            robot.powerIntake(-1);
            //robot.openGrabber();
            sleep(300);
            encoderDrive(-.8,-10,100);
            sleep(200);
            robot.powerIntake(0);
            encoderStrafeRightPID(.5,48,100,0);
            sleep(200);
            encoderDrive(.4,7,100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            //robot.closeGrabber();
            sleep(100);
            robot.powerIntake(0);
            encoderDrive(-1,-17,100);
            sleep(200);
            encoderStrafeLeftPID(.5, 85, 2000,0);
            sleep(200);
            //robot.armLift.setPower(.35);
            //sleep(500);
            encoderDrive(.4, 15, 100);

            //while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            //sleep(500);
            encoderDrive(-1,-60,2000);
            //robot.openGrips();
            //robot.openGrabber();
            encoderStrafeRightPID(.5,40,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrivePID(.7,25,100,0);
            encoderStrafeRightPID(.5,15,100,0);
        }


    }
}


