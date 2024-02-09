package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_ARM_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_ARM_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_ARM_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements Updatable {
    private final Servo claw;
    private final Servo armLeft;
    private final Servo armRight;
    private Telemetry telemetry;
    PController clawController = new PController(CLAW_KP);
    private double clawControllerTarget;
    PController armController = new PController(CLAW_ARM_KP);
    private double armControllerTarget = -1;

    public Claw(HardwareMap hardwareMap) {
        this.claw = hardwareMap.get(Servo.class, CLAW_NAME);
        this.armLeft = hardwareMap.get(Servo.class, CLAW_ARM_LEFT_NAME);
        this.armRight = hardwareMap.get(Servo.class, CLAW_ARM_RIGHT_NAME);
        this.armRight.setDirection(Servo.Direction.REVERSE);
        this.setArmPosition(PICKUP_ARM_MAX);
        this.close();
    }

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public void open() {
        this.clawControllerTarget = CLAW_MAX;
    }

    public void openSync() {
        this.clawControllerTarget = CLAW_MAX;
        while (Math.abs(this.claw.getPosition() - CLAW_MAX) > 0.001) {
            this.update();
        }
    }

    public void close() {
        this.clawControllerTarget = CLAW_MIN;
    }

    public void setArmPosition(double target) {
        target = Math.min(PICKUP_ARM_MAX, Math.max(PICKUP_ARM_MIN, target));
        this.armLeft.setPosition(target);
        this.armRight.setPosition(target);
    }

    public void setArmPositionAsync(double armControllerTarget) {
        this.armControllerTarget = armControllerTarget;
    }

    public boolean isArmAtPosition() {
        return this.armController.atSetPoint();
    }
    public void update() {
        this.clawController.setSetPoint(this.clawControllerTarget);
        this.clawController.setTolerance(0.001);
        this.clawController.setP(CLAW_KP);

        this.armController.setSetPoint(this.armControllerTarget);
        this.armController.setTolerance(0.008);
        this.armController.setP(CLAW_ARM_KP);

        if (!this.clawController.atSetPoint()) {
            double output = 0;
            output = Math.max(-1 * CLAW_MAX, Math.min(CLAW_MAX, this.clawController.calculate(claw.getPosition())));
            this.claw.setPosition(this.claw.getPosition() + output);
        }

        if (this.armControllerTarget > 0 && !this.armController.atSetPoint()) {
            double output = 0;
            output = Math.max(-1 * PICKUP_ARM_MAX, Math.min(PICKUP_ARM_MAX, this.armController.calculate(armLeft.getPosition())));
            this.armLeft.setPosition(this.armLeft.getPosition() + output);
            this.armRight.setPosition(this.armRight.getPosition() + output);
        }
    }
}
