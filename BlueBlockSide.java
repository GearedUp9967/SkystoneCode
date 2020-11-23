package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;


//@Autonomous(name="BlueBlockSide")
public class BlueBlockSide extends BaseAutonomous {
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
            robot.openGrips();
            encoderDrive(-.5,-5,100);
            sleep(200);
            encoderDrive(0.7, 27, 100);
            encoderDrive(.4,5,100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-1, -13, 100);
            sleep(200);
            encoderStrafeLeftPID(.5, 101, 2000,0);
            sleep(200);
            robot.openGrips();
            robot.armLift.setPower(.4);
            sleep(350);
            encoderDrive(.4, 15, 100);

            robot.armLift.setPower(0);

            robot.closeGrips();
            sleep(500);
            encoderDrive(-.7, -29, 2000);
            robot.forward(0, -0.75);
            sleep(1500);
            encoderDrive(1, 35, 100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1, -35, 1000);
        }
        else if(pos== StoneVision.Position.MIDDLE) {
            encoderStrafeRightPID(.5, 27, 100, 0);
            robot.openGrips();
            encoderDrive(.5, -5, 100);
            encoderDrive(0.7, 27, 100);
            encoderDrive(.4, 5, 100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-1, -12, 100);
            sleep(200);
            encoderStrafeLeftPID(.5, 88, 1000, 0);
            sleep(200);
            encoderDrive(.8, 10, 100);
            robot.openGrabber();
            sleep(300);
            encoderDrive(-.8, -10, 100);
            sleep(200);
            encoderStrafeRightPID(.5, 57, 100, 0);
            sleep(200);
            encoderDrive(.4, 12, 100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1, -13, 100);
            sleep(200);
            encoderStrafeLeftPID(.5, 93, 2000, 0);
            sleep(200);
            robot.openGrips();
            robot.armLift.setPower(.4);

            sleep(350);
            encoderDrive(.4, 15, 100);

            robot.armLift.setPower(0);

            robot.closeGrips();
            sleep(500);
            encoderDrive(-.7, -29, 2000);
            robot.forward(0, -0.75);
            sleep(1500);
            encoderDrive(1, 35, 100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1, -35, 1000);
        }
        else if(pos== StoneVision.Position.LEFT)
        {
            encoderStrafeRightPID(.4,19,100,0);
            robot.openGrips();
            encoderDrive(-.2,-8,100);
            encoderDrive(0.7, 27, 100);
            encoderDrive(.4,5,100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-1, -12, 100);
            sleep(200);
            encoderStrafeLeftPID(.5,80,1000,0);
            sleep(200);
            encoderDrive(.8,10,100);
            robot.openGrabber();
            sleep(300);
            encoderDrive(-.8,-10,100);
            sleep(200);
            encoderStrafeRightPID(.5,49,100,0);
            sleep(200);
            encoderDrive(.4,12,100);
            robot.closeGrabber();
            sleep(800);
            encoderDrive(-1,-13,100);
            sleep(200);
            encoderStrafeLeftPID(.5, 85, 2000,0);
            sleep(200);
            robot.armLift.setPower(.4);
            sleep(350);
            encoderDrive(.4, 15, 100);

            robot.armLift.setPower(0);

            robot.closeGrips();
            sleep(500);
            encoderDrive(-.7,-29,2000);
            robot.forward(0,-0.75);
            sleep(1500);
            encoderDrive(1,35,100);
            robot.openGrabber();
            robot.openGrips();
            robot.armLift.setPower(-.2);
            encoderDrive(-1, -35, 1000);
        }


    }
}


