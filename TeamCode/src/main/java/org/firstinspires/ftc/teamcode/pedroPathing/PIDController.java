package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double setpoint;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getKp() {
        return Kp;
    }

    public void setKp(double Kp) {
        this.Kp = Kp;
    }

    public double getKi() {
        return Ki;
    }

    public void setKi(double Ki) {
        this.Ki = Ki;
    }

    public double getKd() {
        return Kd;
    }

    public void setKd(double Kd) {
        this.Kd = Kd;
    }

    public double update(double measurement) {
        double error = setpoint - measurement;

        double dt = timer.seconds();
        if (dt > 0.1) { // prevent large dt on first loop
            dt = 0;
            integralSum = 0; // reset integral
        }
        timer.reset();

        integralSum += error * dt;

        double derivative = (error - lastError) / dt;
        if(Double.isNaN(derivative) || Double.isInfinite(derivative)) {
            derivative = 0;
        }
        lastError = error;

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}