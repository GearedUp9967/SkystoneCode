package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareWazzo {
    public DcMotorEx frontleft;
    public DcMotorEx rearleft;
    public DcMotorEx frontright;
    public DcMotorEx rearright;
    public DcMotorEx spool;
    public DcMotorEx arm;
    public DcMotorEx leftintake;
    public DcMotorEx rightintake;

    public Servo grabber;
    public Servo puller;
    public Servo cap;

    public Servo leftblock;
    public Servo rightblock;
    public Servo rightclamp;
    public Servo leftclamp;




    public static double GRABBER_OPEN=.75;
    public static double GRABBER_CLOSED=.55;
    public static double GRABBER_FULLY_CLOSED=.2;


    public static double PULLER_OPEN=0;
    public static double PULLER_CLOSED=.8;

    public static double CAP_OPEN=.85;
    public static double CAP_CLOSED=.97;

    public static double LEFTBLOCK_UP=0.8;
    public static double LEFTBLOCK_DOWN=1;
    public static double LEFTBLOCK_RESET=0.4;


    public static double RIGHTBLOCK_UP=.9;
    public static double RIGHTBLOCK_DOWN=.45;
    public static double RIGHTBLOCK_RESET=.2;

    public static double RIGHTCLAMP_OPEN=.28;
    public static double RIGHTCLAMP_CLOSED=.17;

    public static double LEFTCLAMP_OPEN=.6;
    public static double LEFTCLAMP_CLOSED=.75;

    HardwareMap hwMap;

    public BNO055IMU imu;

    /* Constructor */
    public HardwareWazzo(){
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

        spool =(DcMotorEx) hwMap.dcMotor.get("spool");
        leftintake =(DcMotorEx) hwMap.dcMotor.get("leftintake");
        rightintake =(DcMotorEx) hwMap.dcMotor.get("rightintake");
        arm = (DcMotorEx) hwMap.dcMotor.get("arm");

        grabber = hwMap.servo.get("grabber");
        puller = hwMap.servo.get("puller");
        cap = hwMap.servo.get("cap");

        leftblock = hwMap.servo.get("leftblock");
        rightblock = hwMap.servo.get("rightblock");
        leftclamp = hwMap.servo.get("leftclamp");
        rightclamp = hwMap.servo.get("rightclamp");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearright.setDirection(DcMotor.Direction.REVERSE);

        leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);


        frontleft.setPower(0);
        rearleft.setPower(0);
        frontright.setPower(0);
        rearright.setPower(0);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        resetLeft();
        resetRight();
        openGrabber();
        openPuller();
        closeCap();
    }


    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void powerIntake(double power) {
        leftintake.setPower(power);
        rightintake.setPower(power);
    }

    public void openGrabber() {
        grabber.setPosition(GRABBER_OPEN);
    }
    public void closeGrabber() {
        grabber.setPosition(GRABBER_CLOSED);
    }
    public void fullyCloseGrabber() {grabber.setPosition(GRABBER_FULLY_CLOSED);}

    public void openPuller () {puller.setPosition(PULLER_OPEN);}
    public void closePuller() {puller.setPosition(PULLER_CLOSED);}

    public void openCap () {cap.setPosition(CAP_OPEN);}
    public void closeCap () {cap.setPosition(CAP_CLOSED);}



    public void resetLeft() {
        leftblock.setPosition(0.45);
        leftclamp.setPosition(0.7);
    }

    public void resetRight() {
        rightblock.setPosition(0.95-.16);
        rightclamp.setPosition(0.2);
    }

    public void EXTREMEresetRight() {
        rightblock.setPosition(1-.10);
        rightclamp.setPosition(0.2);
    }



    public void liftLeft() {
        leftblock.setPosition(0.6);
        leftclamp.setPosition(0.7);
    }

    public void liftRight() {
        rightblock.setPosition(0.7);
        rightclamp.setPosition(0.2);
    }

    public void dropLeft() {
        leftblock.setPosition(0.7);
        leftclamp.setPosition(0.5);
    }

    public void dropRight() {
        rightblock.setPosition(0.8-.16);
        ElapsedTime timer=new ElapsedTime();
        timer.reset();
        while(timer.milliseconds()<200) {}
        rightclamp.setPosition(0.4);

    }


    public void openLeft() {
            leftblock.setPosition(0.83);
            leftclamp.setPosition(0.52);

    }

    public void closeLeft() {
        leftclamp.setPosition(0.7);
        leftblock.setPosition(1);
    }

    public void closeRight() {
        rightclamp.setPosition(0.2);
        rightblock.setPosition(0.4-.16);
    }

    public void openRight() {
        rightblock.setPosition(0.39);
        rightclamp.setPosition(0.48);
    }


    public void openLeftclamp () {leftclamp.setPosition(LEFTCLAMP_OPEN);}
    public void closeLeftclamp () {leftclamp.setPosition(LEFTCLAMP_CLOSED);}


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