package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Odometry")

public class Odometry extends LinearOpMode {

    DcMotor leftEncoder;
    DcMotor rightEncoder;
    public BNO055IMU imu;
    private double COUNTS_PER_INCH=8192.0/(3.14159*4);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder=hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder=hardwareMap.dcMotor.get("rightEncoder");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
        double leftDist;
        double rightDist;

        double posX=0;
        double posY=0;
        double angle=0;

        double width=15;
        double turnRadius=0;

        double leftPos;
        double rightPos;

        double lastLeftPos=0;
        double lastRightPos=0;
        double startLeft=-leftEncoder.getCurrentPosition()/COUNTS_PER_INCH;
        double startRight=rightEncoder.getCurrentPosition()/COUNTS_PER_INCH;
        Orientation angles;
        double lastPosX;
        double lastPosY;

        while(opModeIsActive()) {
            lastPosX=posX;
            lastPosY=posY;
            leftPos=-leftEncoder.getCurrentPosition()/COUNTS_PER_INCH-startLeft;
            rightPos=rightEncoder.getCurrentPosition()/COUNTS_PER_INCH-startRight;
            leftDist=leftPos-lastLeftPos;
            rightDist=rightPos-lastRightPos;
            lastLeftPos=leftPos;
            lastRightPos=rightPos;

            telemetry.addData("Left Inches: ",leftPos);
            telemetry.addData("Right Inches: ",rightPos);

            telemetry.addData("Pos X:", posX);
            telemetry.addData("Pos Y:", posY);
            telemetry.addData("Odometry Angle:",angle*180.0/3.1415926);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("IMU Angle:", angles.firstAngle);
            telemetry.update();


            packet.fieldOverlay().setStrokeWidth(4).setStroke("black").strokeLine(lastPosX,lastPosY,posX,posY);
            dashboard.sendTelemetryPacket(packet);

            if(leftDist==rightDist)
            {
                //Drive move center straight forward
                posX+=Math.cos(angle)*leftDist;
                posY+=Math.sin(angle)*leftDist;
            }

            double turnCenterX=0;
            double turnCenterY=0;

            double turnAngle;

            double tempX;
            double tempY;

            double deltaX;
            double deltaY;
            if(leftDist>rightDist)
            {
                turnAngle=(leftDist-rightDist)/width;
                turnRadius=leftDist/angle-width/2;
                deltaX=rotateVectorX(turnRadius-turnRadius*Math.cos(turnAngle),Math.sin(turnAngle)*turnRadius,angle);
                deltaY=rotateVectorX(turnRadius-turnRadius*Math.cos(turnAngle),Math.sin(turnAngle)*turnRadius,angle);
                posX+=deltaX;
                posY+=deltaY;
            }








        }
    }


    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }
}
