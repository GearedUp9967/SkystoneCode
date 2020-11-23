package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PID {

    double P;
    double I;
    double D;

    double setpoint;

    double integral=0;
    double lastError=0;

    ElapsedTime time=new ElapsedTime();

    PID(double p)
    {
        this(p,0,0);
    }

    PID(double p, double i)
    {
        this(p,i,0);
    }

    PID(double p, double i, double d) {
        P=p;
        I=i;
        D=d;
    }

    public void setTarget(double s){
        setpoint=s;
    }

    public void restart() {
        time.reset();
        integral=0;
        lastError=0;
    }

    public double update(double position) {
        double error=setpoint-position;

        double deltaTime=time.seconds();
        integral+=deltaTime*error;
        double derivative=(error-lastError)/deltaTime;

        lastError=error;
        time.reset();
        return P*error+I*integral+D*derivative;
    }


}
