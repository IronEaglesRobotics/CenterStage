package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    //make each servo
    private Servo rAServo;
    private Servo lAServo;
    private Servo doorServo;
    private Servo wristServo;
    private Slides.Position pos = Slides.Position.DOWN;
    private PController armController;
    private double armControllerTarget;

    public static double armTolerance = 0.03;
    public static double armSpeed = 0.022;
    private double armPos;
    private double armTempPos;
    private double doorPos;
    private double wristPos;
    public static double ARM_KP = 0.08;

    public static double doorOpenPos = 0.09;
    public static double doorClosePos = 0.26;

    public static double armStart = 0.24;
    public static double armScore = 0.95;
    public static double wristStart = 0.29;
    public static double wristScore = 0.5;

    public enum Position {
        INTAKE, SCORE
    }

    public enum DoorPosition {
        OPEN, CLOSE
    }

    public Arm(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "Wrist Servo");
        doorServo = hardwareMap.get(Servo.class, "Door Servo");
        lAServo = hardwareMap.get(Servo.class, "Left Arm Servo");
        rAServo = hardwareMap.get(Servo.class, "Right Arm Servo ");
//        lAServo.setDirection(Servo.Direction.REVERSE);
        rAServo.setDirection(Servo.Direction.REVERSE);
        doorServo.setDirection(Servo.Direction.REVERSE);
//        wristServo.setDirection(Servo.Direction.REVERSE);

        this.armController = new PController(ARM_KP);

        setArmPos(Position.INTAKE);
        lAServo.setPosition(armStart);
        rAServo.setPosition(armStart);
//        armTempPos = armPos;

        setWristPos(Position.INTAKE);
        setDoor(DoorPosition.CLOSE);
    }

    public void setArmPos(Position pos) {
        this.armControllerTarget = pos == Position.INTAKE
                ? armStart
                : armScore;
//        if (pos == Position.INTAKE) {
//            armPos = armStart;
//        } else if (pos == Position.SCORE) {
//            armPos = armScore;
//        }
    }
    public boolean armAtPosition() {
        return armController.atSetPoint();
//        return armTempPos == armPos;
    }

    public void setWristPos(Position pos) {
        if (pos == Position.INTAKE) {
            wristPos = wristStart;
        } else if (pos == Position.SCORE) {
            wristPos = wristScore;
        }
    }

    public void setDoor(DoorPosition pos) {
        if (pos == DoorPosition.OPEN) {
            doorPos = doorOpenPos;
        } else if (pos == DoorPosition.CLOSE) {
            doorPos = doorClosePos;
        }
    }

    public String getTelemetry() {
        return String.format("Wrist: %s\nDoor: %s\nLeft Arm: %s\nRight Arm: %s\nError: %s",
                wristServo.getPosition(), doorServo.getPosition(), lAServo.getPosition(), rAServo.getPosition(), armController.getPositionError());
        //return "Wrist Servo: "+wristServo.getPosition()+"\nLeft Arm Servo: "+lAServo.getPosition()+"\nRight Arm Servo: "+rAServo.getPosition()+"\nDoor Servo: "+ doorServo.getPosition();
    }

    public void update() {
        this.armController.setSetPoint(this.armControllerTarget);
        this.armController.setTolerance(armTolerance);
        this.armController.setP(ARM_KP);

        double output = 0;
        if (!this.armController.atSetPoint()) {
            output = Math.max(-1 * armScore, Math.min(armScore, this.armController.calculate(lAServo.getPosition())));

            this.lAServo.setPosition(this.lAServo.getPosition() + output);
            this.rAServo.setPosition(this.lAServo.getPosition() + output);
        } else {
            lAServo.setPosition(armControllerTarget);
            rAServo.setPosition(armControllerTarget);
        }
//        if (Math.abs(armTempPos-armPos) <= armTolerance) {
//            armTempPos = armPos;
//        } else {
//            if (armTempPos < armPos) {
//                armTempPos += armSpeed;
//            } else if (armTempPos > armPos) {
//                armTempPos -= armSpeed;
//            }
//        }
//        lAServo.setPosition(armTempPos);
//        rAServo.setPosition(armTempPos);

        doorServo.setPosition(doorPos);
        wristServo.setPosition(wristPos);
    }
}
