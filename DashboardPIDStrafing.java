package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Dashboard PID Strafing")
public class DashboardPIDStrafing extends LinearOpMode {
    HardwareMike robot=new HardwareMike();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            double angle=robot.getAngle();

            robot.right(StraightStrafingPIDConstants.power,-StraightStrafingPIDConstants.P*angle);
            dashboardTelemetry.addData("Angle",angle);
            dashboardTelemetry.update();
        }
    }
}
