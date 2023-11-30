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


    public enum Position {
        INTAKE, SCORE;
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

        setArmPos(Position.INTAKE);
        setWristPos(Position.INTAKE);
    }

    public static double doorOpenpos = 0.5;
    public static double doorClosePos = 0.85;

    public static double armStart = 0.22;
    public static double armScore = 1;
    public static double wristStart = 0.9;
    public static double wristScore = 0.98;

    public void setArmPos(Position pos) {
       if (pos == Position.INTAKE) {
           rAServo.setPosition(armStart);
           lAServo.setPosition(armStart);
       } else if (pos == Position.SCORE) {
           rAServo.setPosition(armScore);
           lAServo.setPosition(armScore);
       }
    }

    public void setWristPos(Position pos) {
        if (pos == Position.INTAKE) {
            wristServo.setPosition(wristStart);
        } else if (pos == Position.SCORE) {
            wristServo.setPosition(wristScore);
        }
    }

    public void setDoor(DoorPos.DoorPosition pos) {
        if (pos == DoorPos.DoorPosition.OPEN) {
            dServo.setPosition(doorOpenpos);
        } else if (pos == DoorPos.DoorPosition.CLOSE) {
            dServo.setPosition(doorClosePos);
        }
    }

    public String getTelemetry() {
        return "Wrist Servo: "+wristServo.getPosition()+"Left Arm Servo: "+lAServo.getPosition()+"Right Arm Servo: "+rAServo.getPosition()+"Door Servo: "+dServo.getPosition();
    }

    public void update() {
    }
}
