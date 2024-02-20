package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class endGame_Mechs {
    private Servo servo;
    private Servo hang1;
    private Servo hang2;
    private DcMotor hang;
    public static double initPos = 0.42;
    public static double launchPos = 0.8;
    public static double finger_out = 0.5;
    public static double finger_in = 0.2;
    public static int initHang = -1000;
    public static double hold = 0.8;
    public static double release = 0.5;
    public static double hold2 = 0.8;
    public static double release2 = 0.8;

    public static int HangUp = -860;
    public static int Hanged = -30;
    private int target = 0;
    public static int down = 0;

    public static int manualSpeed = 50;



    public endGame_Mechs(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "Drone");
        this.servo.setPosition(initPos);
        this.servo = hardwareMap.get(Servo.class, "Finger");
        this.servo.setPosition(finger_out);
        this.hang = hardwareMap.get(DcMotor.class, "Hang");
        this.hang.setTargetPosition(0);
        this.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hang.setDirection(DcMotorSimple.Direction.REVERSE);
        this.hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        this.hang1 = hardwareMap.get(Servo.class, "Hanger 1");
//        this.hang1.setPosition(hold);
//        this.hang2 = hardwareMap.get(Servo.class, "Hanger 2");
//        this.hang2.setPosition(hold);

    }

    public void launch() {
        this.servo.setPosition(launchPos);
    }

    public void reset() {
        this.servo.setPosition(initPos);
    }


    public void Finger_in() {this.servo.setPosition(finger_in);}

    public void Finger_out () {this.servo.setPosition(finger_out);}


    public void hang_init_pos(){
        this.hang.setTargetPosition(0);
        this.hang.setPower(1);
    }


    public void hang_final_pos(){
        this.hang.setTargetPosition(2250);
        this.hang.setPower(1);
    }



}


