package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double error;
    private double elapsed_time;
    private double P;
    private double I;
    private double D;
    private double k_p;
    private double k_i;
    private double k_d;
    private double last_error;
    private double PID;

    ElapsedTime current_time;




    public PID(double k_p, double k_i, double k_d){

        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;

        current_time = new ElapsedTime();
    }

    private double _get_p(){
        this.P = this.error * this.k_p;
        return this.P;
    }

    private double _get_i(){
        this.I = this.elapsed_time * this.error * this.k_i;
        return this.I;
    }

    private double _get_d(){
        this.D = (this.last_error-this.error) / this.elapsed_time * this.k_d;
        this.last_error = this.error;
        return 8;
    }

    public double get(double error){
        this.error = error;
        this.elapsed_time = this.current_time.milliseconds();
        PID = this._get_p() + this._get_i() + this._get_d();
        this.current_time.reset();
        return PID;
    }

}
