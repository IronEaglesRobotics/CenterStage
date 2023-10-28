package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_ARM_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.PICKUP_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_ARM_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo claw;
    private final Servo armLeft;
    private final Servo armRight;

    public Claw(HardwareMap hardwareMap) {
        this.claw = hardwareMap.get(Servo.class, CLAW_NAME);
        this.armLeft = hardwareMap.get(Servo.class, CLAW_ARM_LEFT_NAME);
        this.armRight = hardwareMap.get(Servo.class, CLAW_ARM_RIGHT_NAME);
        this.armRight.setDirection(Servo.Direction.REVERSE);
    }

    public void open() {
        this.claw.setPosition(CLAW_MAX);
    }

    public void close() {
        this.claw.setPosition(CLAW_MIN);
    }

    public void up() {
        this.setArmPosition(PICKUP_ARM_MAX);
    }

    public void down() {
        this.setArmPosition(PICKUP_ARM_MIN);
    }

    public void setArmPosition(double target) {
        this.armLeft.setPosition(target);
        this.armRight.setPosition(target);
    }
}
