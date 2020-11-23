package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Speed Test")
public class SpeedTest extends LinearOpMode {
    HardwareMike robot=new HardwareMike();



    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_FOOT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*12;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.forward(SpeedTestConstants.SpeedTestSpeed,0);

            dashboardTelemetry.addData("Speed (ft/s)", robot.rearleft.getVelocity());
            dashboardTelemetry.update();
        }
    }
}
