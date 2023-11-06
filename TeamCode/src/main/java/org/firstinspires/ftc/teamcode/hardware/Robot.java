package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Configurables.ARMDOWN;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARMUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.BIGOPEN;
import static org.firstinspires.ftc.teamcode.util.Configurables.CLOSE;
import static org.firstinspires.ftc.teamcode.util.Configurables.LOCK;
import static org.firstinspires.ftc.teamcode.util.Configurables.LOCKSPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.OPEN;
import static org.firstinspires.ftc.teamcode.util.Configurables.PICKUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.UNLOCK;
import static org.firstinspires.ftc.teamcode.util.Configurables.UNLOCKSPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.WRISTPICKUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.WRISTSCORE;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFTARM;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFTHANG;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHTARM;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHTHANG;
import static org.firstinspires.ftc.teamcode.util.Constants.WRIST;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

import lombok.Getter;

public class Robot {
    @Getter
    private MecanumDrive drive;
//    private Hang hang;
    @Getter private Intake intake;
    @Getter private Arm arm;
    @Getter private Wrist wrist;
    @Getter private Claw claw;
    @Getter private Hang hang;

    public Robot init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.hang = new Hang().init(hardwareMap);
        this.intake = new Intake().init(hardwareMap);
        this.arm = new Arm().init(hardwareMap);
        this.wrist = new Wrist().init(hardwareMap);
        this.claw = new Claw().init(hardwareMap);
        return this;
    }

    public static class Intake{
        private DcMotor intake = null;

        public Intake init(HardwareMap hardwareMap) {
            this.intake = hardwareMap.dcMotor.get("intake");
            this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return this;
        }

        public void spinIn() {this.intake.setPower(0.7);}

        public void spinOut() {this.intake.setPower(-0.7);}

        public void stop() {this.intake.setPower(0);}

    }

    public static class Hang {
        private Servo hangLeft;
        private Servo hangRight;

        public Hang init(HardwareMap hardwareMap) {
            this.hangLeft = hardwareMap.get(Servo.class, LEFTHANG);
            this.hangRight = hardwareMap.get(Servo.class, RIGHTHANG);
            this.hangLeft.setPosition(LOCKSPEED);
            this.hangRight.setPosition(LOCK);
            return this;
        }

        public void lock() {
            this.hangLeft.setPosition(LOCKSPEED);
            this.hangRight.setPosition(LOCK);
        }

        public void release() {
            this.hangLeft.setPosition(UNLOCKSPEED);
            this.hangRight.setPosition(UNLOCK);
        }
    }

    public static class Arm {
        private Servo leftArm;
        private Servo rightArm;


        public Arm init(HardwareMap hardwareMap){
            this.leftArm = hardwareMap.get(Servo.class, LEFTARM);
            this.rightArm = hardwareMap.get(Servo.class, RIGHTARM);
            this.rightArm.setDirection(Servo.Direction.REVERSE);
            this.rightArm.setPosition(ARMUP);
            this.leftArm.setPosition(ARMUP);
            return this;
        }

        public void pickup() {
            this.leftArm.setPosition(PICKUP);
            this.rightArm.setPosition(PICKUP);
        }

        public void rest() {
            this.leftArm.setPosition(ARMDOWN);
            this.rightArm.setPosition(ARMDOWN);
        }

        public void scoreArm() {
            this.leftArm.setPosition(ARMUP);
            this.rightArm.setPosition(ARMUP);
        }
    }

    public static class Wrist {
        private Servo wrist;

        public Wrist init(HardwareMap hardwareMap){
            this.wrist = hardwareMap.get(Servo.class, WRIST);
            this.wrist.setPosition(WRISTPICKUP);
            return this;
        }

        public void wristPickup() {
            this.wrist.setPosition(WRISTPICKUP);
        }

        public void wristScore() {
            this.wrist.setPosition(WRISTSCORE);
        }
    }

    public static class Claw {
        private Servo claw;

        public Claw init (HardwareMap hardwareMap) {
            this.claw = hardwareMap.get(Servo.class, CLAW);
            return this;
        }

        public void close() {
            this.claw.setPosition(CLOSE);
        }

        public void open() {
            this.claw.setPosition(OPEN);
        }

        public void openScore() {
            this.claw.setPosition(BIGOPEN);
        }

    }

}