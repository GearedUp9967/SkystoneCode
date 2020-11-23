package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;

@TeleOp(name="Triangle")

public class Triangle extends OpMode {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

    BNO055IMU imu;

    public void init() {

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

        motor1 = hardwareMap.dcMotor.get("m1");
        motor2 = hardwareMap.dcMotor.get("m2");
        motor3 = hardwareMap.dcMotor.get("m3");
    }


    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    boolean FIELD_CENTRIC=true;

    public void powerleft(double power){
        motor3.setPower(-power);
        motor2.setPower(-power);
        motor1.setPower(-power);
    }

    //m1=b
    //m2=c
    //m3=a
    double xx;
    double yy;
    double Tx;
    double Ty;

    double turn=0;
    @Override
    public void loop() {
        xx=gamepad1.left_stick_x;
        yy=gamepad1.left_stick_y;
        double adjust=0;
        if(FIELD_CENTRIC)
        {
            adjust=getAngle()*Math.PI/180;
        }

        Tx=rotateVectorX(xx,yy,adjust);
        Ty=rotateVectorY(xx,yy,adjust);




        if (gamepad1.right_bumper) {
            turn=.4;
        }

        else if (gamepad1.left_bumper) {
            turn=-.4;
        }
        else
        {
            turn=0;
        }

        motor1.setPower(Ty*(-2.0/3.0)+turn);
        motor2.setPower((Ty-Math.sqrt(3)*Tx)/3+turn);
        motor3.setPower((Ty+Math.sqrt(3)*Tx)/3+turn);
    }

    public double rotateVectorX(double x, double y, double angle) {
        return x* Math.cos(angle)-y* Math.sin(angle);
    }
    public double rotateVectorY(double x, double y, double angle) {
        return x* Math.sin(angle)+y* Math.cos(angle);
    }

}
