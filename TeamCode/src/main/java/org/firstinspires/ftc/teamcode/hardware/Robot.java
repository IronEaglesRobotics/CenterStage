package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Configurables.LOCK;
import static org.firstinspires.ftc.teamcode.util.Configurables.LOCKSPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.UNLOCK;
import static org.firstinspires.ftc.teamcode.util.Configurables.UNLOCKSPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

public class Robot {
    private MecanumDrive drive;
    private Hang hang;
    private Intake intake;

    public Robot init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.hang = new Hang().init(hardwareMap);

        return this;
    }

    public Intake getIntake() {return this.intake;}

    public Hang getHang() {return this.hang;}

    public MecanumDrive getDrive() {return this.drive;}

    public static class Intake {
        private DcMotor intake = null;

        public Intake init(HardwareMap hardwareMap) {
            this.intake = hardwareMap.dcMotor.get("intake");
            this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return this;
        }

        public void spinIn() {this.intake.setPower(0.7);}

        public void spinOut() {this.intake.setPower(-0.7);}
    }

    public static class Hang {
        private Servo hangLeft;
        private Servo hangRight;

        public Hang init(HardwareMap hardwareMap) {
            this.hangLeft = hardwareMap.get(Servo.class, LEFT);
            this.hangRight = hardwareMap.get(Servo.class, RIGHT);
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

}