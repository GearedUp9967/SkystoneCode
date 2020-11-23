package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name="BluePark")
public class BluePark extends BaseAutonomous {
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

        phoneCam.startStreaming(3264, 2448, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        pos = vision.getPos();
        //vision.disable();

        if (pos == StoneVision.Position.RIGHT) {
            encoderStrafeRight(.1, 10, 100);
            encoderDrive(-.2, -8, 100);
            sleep(200);
            encoderDrive(0.1, 32, 100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-0.2, -12, 100);
            sleep(200);
            pidTurn(90, 0.005);
            encoderDrive(.1, 81, 2000);
            sleep(200);
            robot.openGrabber();
            sleep(200);
            encoderDrive(-.2, -40, 100);
        } else if (pos == StoneVision.Position.MIDDLE) {
            encoderDrive(0.1, 32, 100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-0.2, -12, 100);
            sleep(200);
            pidTurn(90, 0.005);
            encoderDrive(.1, 73, 2000);
            sleep(200);
            robot.openGrabber();
            sleep(200);
            encoderDrive(-.2, -40, 100);
        } else if (pos == StoneVision.Position.LEFT) {
            encoderStrafeLeft(.1, 10, 100);
            encoderDrive(0.1, 32, 100);
            robot.closeGrabber();
            sleep(1000);
            encoderDrive(-0.2, -12, 100);
            sleep(200);
            pidTurn(90, 0.005);
            encoderDrive(.1, 65, 2000);
            sleep(200);
            robot.openGrabber();
            sleep(200);
            encoderDrive(-.2, -40, 100);
        }
    }
}




