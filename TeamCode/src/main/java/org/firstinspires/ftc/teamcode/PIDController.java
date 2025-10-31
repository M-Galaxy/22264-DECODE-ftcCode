package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain

    private double target; // Setpoint
    private double integral; // Integral term accumulation
    private double previousError; // Previous error value

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.target = 0.0;
        this.integral = 0.0;
        this.previousError = 0.0;
    }

    public void setTarget(double target) {
        this.target = target;
        this.integral = 0.0;
        this.previousError = 0.0;
    }

    public double calculateOutput(double currentValue) {
        double error = target - currentValue;

        // Proportional term
        double proportionalTerm = kp * error;

        // Integral term
        integral += error * timer.seconds();
        double integralTerm = ki * integral;

        // Derivative term
        double derivativeTerm = kd * ((error - previousError) / timer.seconds());

        // Calculate the output value
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Update previous error value
        previousError = error;

        //reset the timer
        timer.reset();

        return output;
    }
}