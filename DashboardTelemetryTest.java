package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name="Dashboard")
public class DashboardTelemetryTest extends LinearOpMode {

    HardwareMike robot=new HardwareMike();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive())
        {
            robot.forward(0.2,0);
            dashboardTelemetry.addData("Front Left Encoder",robot.frontleft.getCurrentPosition());
            dashboardTelemetry.addData("Front Right Encoder",robot.frontright.getCurrentPosition());
            dashboardTelemetry.update();

        }
    }
}
