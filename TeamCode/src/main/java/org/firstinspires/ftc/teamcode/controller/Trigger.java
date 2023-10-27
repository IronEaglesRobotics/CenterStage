package org.firstinspires.ftc.teamcode.controller;

public class Trigger {
    double value = 0;
    double currentState;
    double lastState;
    boolean justPressed = false;
    boolean justReleased = false;

    public Trigger() {}

    public void update(double value) {
        lastState = currentState;
        currentState = value;

        justPressed = currentState != 0 && lastState == 0;
        justReleased = currentState == 0 && lastState != 0;

        this.value = value;
    }

    public double getValue() {
        return value;
    }

    public boolean isJustPressed() {
        return justPressed;
    }

    public boolean isJustReleased() {
        return justReleased;
    }
}