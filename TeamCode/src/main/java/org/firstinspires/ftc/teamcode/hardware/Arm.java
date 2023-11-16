package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.robby.Slides;

@Config
public class Arm {

    //make each servo
    private Servo dServo;
    private Servo rAServo;
    private Servo lAServo;
    private Servo wristServo;
    private Slides.Position pos = Slides.Position.DOWN;
    private PController armController;
    private double armControllerTarget;
    private double ARM_KP = 0.001;


    public enum APosition {
        SDOWN, SCORE, SAFTEYDOWN, SAFTEYUP;
    }

    public Arm(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "Wrist Servo");
        dServo = hardwareMap.get(Servo.class, "Door Servo");
        lAServo = hardwareMap.get(Servo.class, "Left Arm Servo");
        rAServo = hardwareMap.get(Servo.class, "Right Arm Servo ");
//        lAServo.setDirection(Servo.Direction.REVERSE);
        rAServo.setDirection(Servo.Direction.REVERSE);
        dServo.setDirection(Servo.Direction.REVERSE);
//        wristServo.setDirection(Servo.Direction.REVERSE);
    }

    public static double armScorePos = 1;
    public static double doorOpenpos = 0.5;
    public static double doorClosePos = 0.85;
    public static double startingWristPos = 0.735;
    public static double startingArmPos = 0.325;
    public static double safteyDownPos = 0.4;
    public static double safteyUpPos = 0.9;
    public static double wristScorePos = 0.98;
    public static double safeWrist = 0.8;

    public void openDoor(DoorPos.DoorPosition pos) {
        if (pos == DoorPos.DoorPosition.OPEN) {
            dServo.setPosition(doorOpenpos);
        } else if (pos == DoorPos.DoorPosition.CLOSE) {
            dServo.setPosition(doorClosePos);
        }
    }

    public void setPos(APosition tape) {
        if (tape == APosition.SDOWN) {
            this.armControllerTarget = startingArmPos;
            lAServo.setPosition(startingArmPos);
            rAServo.setPosition(startingArmPos);
            wristServo.setPosition(startingWristPos);
        } else if (tape == APosition.SCORE) {
            this.armControllerTarget = armScorePos;
            lAServo.setPosition(armScorePos);
            rAServo.setPosition(armScorePos);
            wristServo.setPosition(wristScorePos);
        } else if (tape == APosition.SAFTEYDOWN) {
            lAServo.setPosition(safteyDownPos);
            rAServo.setPosition(safteyDownPos);
            wristServo.setPosition(wristScorePos);
        } else if (tape == APosition.SAFTEYUP) {
            lAServo.setPosition(safteyUpPos);
            rAServo.setPosition(safteyUpPos);
            wristServo.setPosition(wristScorePos);
        }
    }


//    public void setHopper(HopperPos.hopperPos hopper) {
//        if (hopper == HopperPos.hopperPos.UP) {
//                wristServo.setPosition(wristScorePos);
//        } else if (hopper == HopperPos.hopperPos.DOWN) {
//            wristServo.setPosition(startingWristPos);
//        }
//    }

    public String getTelemetry() {
        return "Wrist Servo: "+wristServo.getPosition()+"Left Arm Servo: "+lAServo.getPosition()+"Right Arm Servo: "+rAServo.getPosition()+"Door Servo: "+dServo.getPosition();
    }

    public void update() {
        armController.setP(0.01);
        this.armController.setSetPoint(this.armControllerTarget);

        double output = this.armController.calculate(this.lAServo.getPosition());

        this.lAServo.setPosition(this.lAServo.getPosition() + output);
        this.rAServo.setPosition(this.rAServo.getPosition() + output);
    }
}
