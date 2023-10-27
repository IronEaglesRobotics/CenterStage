package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake { // TODO done in theory, but need to get the actual servo positions &&&& do the Sensor stuff
    private Servo rServo;
    private Servo lServo;
    private DcMotor dcMotor;
    private Position pos = Position.UP;

    public enum Position {
        STACK1, STACK2, STACK3, STACK4, STACK5, UP;

        public Position nextPosition() {
            int nextOne = (this.ordinal() + 1) % Position.values().length;
            return Position.values()[nextOne];
        }

        public Position previousPosition() {
            int backOne = (this.ordinal() - 1) % Position.values().length;
            return Position.values()[backOne];
        }
    }

    //Position
    public static double stack1 = 0.49;
    public static double stack2 = 0.50;
    public static double stack3 = 0.51;
    public static double stack4 = 0.52;
    public static double stack5 = 0.53;
    public static double up = 0.551;
    public static double motorPower = 0;

    public Intake(HardwareMap hardwareMap) {
        lServo = hardwareMap.get(Servo.class, "Left Servo");
        lServo.setDirection(Servo.Direction.REVERSE);
        rServo = hardwareMap.get(Servo.class, "Right Servo");
        dcMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
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
        } else if (stack == Position.UP) {
            lServo.setPosition(up);
            rServo.setPosition(up);
        }
    }

    public void setDcMotor(double pwr) {
        dcMotor.setPower(pwr);
    }

    public String getTelemetry() {
        return "lServo: "+lServo.getPosition()+"rServo: "+rServo.getPosition()+"dcMotor: "+dcMotor.getPower();
    }
}
