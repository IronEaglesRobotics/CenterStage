package org.firstinspires.ftc.teamcode.controller;

public class Button {
    boolean currentState = false;
    boolean lastState = false;
    boolean justPressed = false;
    boolean justReleased = false;

    public Button() {}

    public void update(boolean pressed) {
        lastState = currentState;
        currentState = pressed;

        justPressed = currentState && !lastState;
        justReleased = !currentState && lastState;
    }

    public boolean isPressed() {
        return currentState;
    }

    public boolean isJustPressed() {
        return justPressed;
    }

    public boolean isJustReleased() {
        return justReleased;
    }
}