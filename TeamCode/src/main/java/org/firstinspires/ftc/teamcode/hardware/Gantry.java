package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_CENTER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_SCREW_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_SCREW_POSITIONS;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_X_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gantry extends Slide {
    private final Servo xServo;
    private final Servo armServo;
    private final Servo screwServo;
    private int currentScrewIndex = 0;

    public Gantry(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.xServo = hardwareMap.get(Servo.class, GANTRY_X_NAME);
        this.armServo = hardwareMap.get(Servo.class, GANTRY_ARM_NAME);
        this.screwServo = hardwareMap.get(Servo.class, GANTRY_SCREW_NAME);
    }

    public void setX(double x) {
        this.xServo.setPosition(x);
    }

    public double getX() {
        return this.xServo.getPosition();
    }

    public void armOut() {
        this.armServo.setPosition(GANTRY_ARM_MAX);
    }

    public void armIn() {
        this.armServo.setPosition(GANTRY_ARM_MIN);
    }

    public void resetScrew() {
        this.currentScrewIndex = 0;
        this.screwServo.setPosition(GANTRY_SCREW_POSITIONS[currentScrewIndex]);
    }

    public void deposit() {
        this.screwServo.setPosition(GANTRY_SCREW_POSITIONS[this.currentScrewIndex--]);
    }

    public void center() {
        this.setX(GANTRY_CENTER);
    }
}
