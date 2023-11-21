package org.firstinspires.ftc.teamcode.opmode.autonomous;


import org.firstinspires.ftc.teamcode.vision.Detection;

// Class for every step that the autonomous program will take
public abstract class Step {
    private final double timeout;
    private String telemetry;

    // variables when moving
    public double x;
    public double y;
    public double stepTime;
    double tempTime = stepTime;

    // variables when shooting
    public Detection teamProp;

    // Constructors
    public Step(String telemetry) {
        this.telemetry = telemetry;
        this.timeout = -1;
    }

    public Step(String telemetry, double timeout) {
        this.telemetry = telemetry;
        this.timeout = timeout;
    }

    // Abstract methods to be overrode
    public abstract void start();

    public abstract void whileRunning();

    public abstract void end();

    public abstract boolean isFinished();

    // Return the timeout for the step
    public double getTimeout() {
        return timeout;
    }

    public void setTelemetry(String telemetry) {
        this.telemetry = telemetry;
    }

    // Return the Telemetry for the step
    public String getTelemetry() {
        return telemetry;
    }
}