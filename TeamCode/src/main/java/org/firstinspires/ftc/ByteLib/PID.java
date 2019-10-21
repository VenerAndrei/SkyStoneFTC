package org.firstinspires.ftc.ByteLib;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID  {

    public double Kp;
    public double Ki;
    public double Kd;

    public double I_SumMaxVal;
    public double I_Sum;

    public double last_e;
    public double last_time;

    public double err;

    public PID(){

        //Init all with 0

        this.Kp = 0;
        this.Ki = 0;
        this.Kd = 0;
        this.last_time = 0;
        this.last_e = 0;
    }

    //To change values if you don't like them
    public void setKPID(double p, double i, double d){

        //Change K's with desired values

        this.Kp = p;
        this.Ki = i;
        this.Kd = d;

    }

    // Return the Proportional
    public double P_err(double e){
        return (this.Kp * e);
    }

    // Return the Integral
    public double I_err(double e){

        this.I_Sum = this.I_Sum + e;

        if(this.I_Sum > this.I_SumMaxVal) this.I_Sum = this.I_SumMaxVal;
        if(this.I_Sum < -this.I_SumMaxVal) this.I_Sum = -this.I_SumMaxVal;

        return (this.Ki * this.I_Sum);

    }

    // Return the Integral
    public double D_err(double e,double time_in_milliseconds){
        double DUp = e - last_e;
        double DDown = time_in_milliseconds - last_time;
        double D = DUp/DDown;
        return this.Kd * D;
    }

    public double solvePID(double target, double val, ElapsedTime t){
        double err = target - val;
        return (P_err(err) + I_err(err) + D_err(err,t.milliseconds()));
    }

    public void D_deltaReset(){
        this.last_e = 0;
        this.last_time = 0;
    }
    public void log_info(double target, double val){
        Log.v("PID","Bubu");
    }

}
