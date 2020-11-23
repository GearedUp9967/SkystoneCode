package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


//@Autonomous(name="RedBlockSideIntake")
public class RedBlockSideIntake extends BaseAutonomousIntake {
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

            encoderStrafeLeftPID(1,26,100,0);
            robot.openGrabber();
            //robot.openGrips();
            encoderDrive(-1,-5,100);
            encoderDrive(1, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            robot.closeGrabber();
            sleep(300);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeRightPID(1,82,2000,0);
            sleep(100);
            robot.openGrabber();
            encoderDrive(1,15,100);
            robot.powerIntake(-1);
            sleep(300);
            encoderDrivePID(-1,-15,100,0);
            sleep(100);
            robot.powerIntake(0);
            encoderStrafeLeftPID(1,52,1000,0);
            sleep(100);
            encoderDrive(1,7,100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            robot.closeGrabber();
            sleep(300);
            robot.powerIntake(0);
            encoderDrive(-1,-15,100);
            sleep(200);
            encoderStrafeRightPID(1,82,2000,0);
            sleep(100);
            pidTurn(180,1.0/45.0 ,true);
            //robot.openGrips();
            //robot.armLift.setPower(.35);
            sleep(100);
            encoderDrive(.4, -15, 100);
            robot.arm.setPower(.8);
            sleep(2000);
            robot.openGrabber();
            robot.arm.setPower(-1);
            //robot.closeGrips();
            //while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            sleep(600);
            robot.closeGrabber();
            encoderDrive(-.6,60,2000);
           // robot.openGrips();
            robot.arm.setPower(0);
            //robot.openGrabber();
            //robot.openGrips();
            encoderStrafeLeftPID(.5,-42,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrive(.7,-25,100);
            encoderStrafeLeftPID(.5,-15,100,0);
        }

        else if(pos== StoneVision.Position.MIDDLE)
        {
            encoderStrafeLeftPID(1,34,100,0);
            robot.openGrabber();
            //robot.openGrips();
            encoderDrive(-1,-5,100);
            encoderDrive(1, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            robot.closeGrabber();
            sleep(300);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeRightPID(1,90,2000,0);
            sleep(100);
            robot.openGrabber();
            encoderDrive(1,15,100);
            robot.openGrabber();
            robot.powerIntake(-1);
            sleep(300);
            encoderDrive(-1,-15,100);
            sleep(100);
            robot.powerIntake(0);
            encoderStrafeLeftPID(1,66,1000,0);
            sleep(100);
            encoderDrive(1,7,100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            robot.closeGrabber();
            sleep(300);
            robot.powerIntake(0);
            encoderDrive(-1,-15,100);
            sleep(200);
            encoderStrafeRightPID(1, 90, 2000,0);
            sleep(100);
            //robot.armLift.setPower(.35);
            pidTurn(180,1.0/45.0 ,true);
            sleep(100);
            encoderDrive(.4, -15, 100);
            robot.arm.setPower(.8);
            sleep(2000);
            robot.openGrabber();
            robot.arm.setPower(-1);
            //robot.closeGrips();
            //while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            sleep(600);
            robot.closeGrabber();
            encoderDrive(-.6,60,2000);
            //robot.openGrips();
            robot.arm.setPower(0);
            //robot.openGrabber();
            //robot.openGrips();
            encoderStrafeLeftPID(.5,-42,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrive(.7,-25,100);
            encoderStrafeLeftPID(.5,-15,100,0);
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderStrafeLeftPID(.3,17,100,0);
            robot.openGrabber();
            //robot.openGrips();
            encoderDrive(-.4,-8,100);
            sleep(200);
            encoderDrive(0.7, 27, 100);
            robot.powerIntake(1);
            encoderDrive(.1,10,100);
            robot.closeGrabber();
            sleep(300);
            robot.powerIntake(0);
            encoderDrive(-1, -17, 100);
            sleep(200);
            encoderStrafeRightPID(.5, 104, 2000,0);
            sleep(100);
            //robot.armLift.setPower(.35);
            pidTurn(180,1.0/45.0 ,true);
            sleep(100);
            encoderDrive(.4, -15, 100);
            robot.arm.setPower(.8);
            sleep(2000);
            robot.openGrabber();
            robot.arm.setPower(-1);
            //robot.closeGrips();
            //while(robot.armLift.getCurrentPosition()<1000) { }
            //robot.armLift.setPower(0);

            //robot.closeGrips();
            sleep(600);
            robot.closeGrabber();
            encoderDrive(-.6,60,2000);
            //robot.openGrips();
            robot.arm.setPower(0);
            //robot.openGrabber();
            //robot.openGrips();
            encoderStrafeLeftPID(.5,-42,1000,0);
            //robot.armLift.setPower(-.15);
            encoderDrive(.7,-25,100);
            encoderStrafeLeftPID(.5,-15,100,0);

        }


    }
}

