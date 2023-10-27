package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad gamepad;

    private final Joystick leftStick;
    private final Joystick rightStick;

    private final Button leftStickButton;
    private final Button rightStickButton;

    private final Button dLeft;
    private final Button dRight;
    private final Button dUp;
    private final Button dDown;

    private final Button a;
    private final Button b;
    private final Button x;
    private final Button y;

    private final Button leftBumper;
    private final Button rightBumper;

    private final Button back;
    private final Button start;

    private final Trigger leftTrigger;
    private final Trigger rightTrigger;

    private final Button touchpad;

    private final Button allButtons;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftStick = new Joystick();
        rightStick = new Joystick();

        leftStickButton = new Button();
        rightStickButton = new Button();

        dLeft = new Button();
        dRight = new Button();
        dUp = new Button();
        dDown = new Button();

        a = new Button();
        b = new Button();
        x = new Button();
        y = new Button();

        leftBumper = new Button();
        rightBumper = new Button();

        back = new Button();
        start = new Button();

        leftTrigger = new Trigger();
        rightTrigger = new Trigger();

        touchpad = new Button();

        allButtons = new Button();
    }


    public void update() {
        leftStick.update(gamepad.left_stick_x, -gamepad.left_stick_y);
        rightStick.update(gamepad.right_stick_x, -gamepad.right_stick_y);

        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);

        dLeft.update(gamepad.dpad_left);
        dRight.update(gamepad.dpad_right);
        dUp.update(gamepad.dpad_up);
        dDown.update(gamepad.dpad_down);

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);

        back.update(gamepad.back);
        start.update(gamepad.start);

        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);

        touchpad.update(gamepad.touchpad);

        allButtons.update(leftStickButton.isPressed() || rightStickButton.isPressed() || dLeft.isPressed() || dRight.isPressed() || dUp.isPressed() || dDown.isPressed() || a.isPressed() || b.isPressed() || x.isPressed() || y.isPressed() || leftBumper.isPressed() || rightBumper.isPressed() || back.isPressed() || start.isPressed() || leftTrigger.getValue() > 0 || rightTrigger.getValue() > 0 || touchpad.isPressed());
    }

    public Joystick getLeftStick() {
        return leftStick;
    }
    public Joystick getRightStick() {
        return rightStick;
    }

    public Button getLeftStickButton() {
        return leftStickButton;
    }
    public Button getRightStickButton() {
        return rightStickButton;
    }

    public Button getDLeft() {
        return dLeft;
    }
    public Button getDRight() {
        return dRight;
    }
    public Button getDUp() {
        return dUp;
    }
    public Button getDDown() {
        return dDown;
    }

    public Button getA() {
        return a;
    }
    public Button getB() {
        return b;
    }
    public Button getX() {
        return x;
    }
    public Button getY() {
        return y;
    }

    public Button getLeftBumper() {
        return leftBumper;
    }
    public Button getRightBumper() {
        return rightBumper;
    }

    public Button getBack() {
        return back;
    }
    public Button getStart() {
        return start;
    }

    public Trigger getLeftTrigger() {
        return leftTrigger;
    }
    public Trigger getRightTrigger() {
        return rightTrigger;
    }

    public Button getTouchpad() {
        return touchpad;
    }

    public Button getAllButtons() {
        return allButtons;
    }

    public void rumble() {
        gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void rumble(int milliseconds) {
        gamepad.rumble(milliseconds);
    }

    public void rumbleBlips(int blips) {
        gamepad.rumbleBlips(blips);
    }

    public void stopRumble() {
        gamepad.stopRumble();
    }

    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    public void setColor(int r, int g, int b) {
        gamepad.setLedColor(r, g, b, -1);
    }
    public void setColor(int r, int g, int b, int milliseconds) {
        gamepad.setLedColor(r, g, b, milliseconds);
    }
}
