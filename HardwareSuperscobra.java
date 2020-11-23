package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareSuperscobra {
    public DcMotorEx frontleft;
    public DcMotorEx rearleft;
    public DcMotorEx frontright;
    public DcMotorEx rearright;

    HardwareMap hwMap;

    public BNO055IMU imu;

    /* Constructor */
    public HardwareSuperscobra(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
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
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        frontleft =(DcMotorEx) hwMap.dcMotor.get("frontleft");
        rearleft =(DcMotorEx) hwMap.dcMotor.get("rearleft");
        frontright =(DcMotorEx) hwMap.dcMotor.get("frontright");
        rearright =(DcMotorEx) hwMap.dcMotor.get("rearright");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearright.setDirection(DcMotor.Direction.REVERSE);



        frontleft.setPower(0);
        rearleft.setPower(0);
        frontright.setPower(0);
        rearright.setPower(0);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void stop() {
        rearleft.setPower(0);
        rearright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    public void forward(double power,double turn) {
        this.frontleft.setPower(power -turn);
        this.rearleft.setPower(power  -turn);
        this.frontright.setPower(power+turn);
        this.rearright.setPower(power +turn);
    }


    public void backward(double power) {
        this.frontleft.setPower(-power);
        this.rearleft.setPower(-power);
        this.frontright.setPower(-power);
        this.rearright.setPower(-power);
    }

    public void right(double power, double turn) {
        this.frontleft.setPower(power  -turn);
        this.rearleft.setPower(-power  -turn);
        this.frontright.setPower(-power+turn);
        this.rearright.setPower(power  +turn);
    }


    public void left(double power, double turn) {

        this.frontleft.setPower(-power -turn);
        this.rearleft.setPower(power   -turn);
        this.frontright.setPower(power +turn);
        this.rearright.setPower(-power +turn);
    }
}