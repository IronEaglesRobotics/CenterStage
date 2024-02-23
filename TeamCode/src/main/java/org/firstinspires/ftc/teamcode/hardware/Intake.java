package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private Servo rServo;
    private Servo lServo;
    private DcMotor dcMotor;
    private Position pos = Position.UP;

    public enum Position {
        STACK1, STACK2, STACK3, STACK4, STACK5, STACK6, UP;

        public Position nextPosition() {
            int nextOne = (this.ordinal() + 1) % Position.values().length;
            return Position.values()[nextOne];
        }

        public Position previousPosition() {
            int backOne =Math.max(0,(this.ordinal() - 1) % Position.values().length);
            return Position.values()[backOne];
        }
    }

    //Position
    public static double stack1 = 0.015;
    public static double stack2 = 0.026;
    public static double stack3 = 0.065;
    public static double stack4 = 0.09;
    public static double stack5 = 0.1;

    public static double stack6 = 0.13;
    public static double up = 0.30;
    public static double motorPower = 0;

    public Intake(HardwareMap hardwareMap, Position up) {
        lServo = hardwareMap.get(Servo.class, "LeftIntake");
        lServo.setDirection(Servo.Direction.REVERSE);
        rServo = hardwareMap.get(Servo.class, "Right Intake Servo");
        dcMotor = hardwareMap.get(DcMotor.class, "Intakemotor");

    }

    public void setpos(Position stack) {
        if (stack == Position.STACK1) {
            lServo.setPosition(stack1);
            rServo.setPosition(stack1);
        } else if (stack == Position.STACK2) {
            lServo.setPosition(stack2);
            rServo.setPosition(stack2);
        } else if (stack == Position.STACK3) {
            lServo.setPosition(stack3);
            rServo.setPosition(stack3);
        } else if (stack == Position.STACK4) {
            lServo.setPosition(stack4);
            rServo.setPosition(stack4);
        } else if (stack == Position.STACK5) {
            lServo.setPosition(stack5);
            rServo.setPosition(stack5);
        } else if (stack == Position.STACK6) {
            lServo.setPosition(stack6);
            rServo.setPosition(stack6);
        }
        else if (stack == Position.UP) {
            lServo.setPosition(up);
            rServo.setPosition(up);
        }
    }

    public void incrementPos() {
        pos = pos.nextPosition();
    }

    public void decrementPos() {
        pos = pos.previousPosition();
    }

    public void setDcMotor(double pwr) {
        dcMotor.setPower(pwr);
        if (pwr >= 0.01) {
            this.setpos(this.pos);
        }
        else {
            this.setpos(Position.UP);
        }
    }

    public double getPower() {
        return dcMotor.getPower();
    }

    public String getTelemetry() {
        return "lServo: "+lServo.getPosition()+"rServo: "+rServo.getPosition()+"dcMotor: "+dcMotor.getPower();
    }
}
