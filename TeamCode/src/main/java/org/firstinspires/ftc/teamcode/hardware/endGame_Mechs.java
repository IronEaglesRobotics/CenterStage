package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class endGame_Mechs {
    private Servo servo;
    private Servo hang1;
    private Servo hang2;
    public static double initPos = 0.8;
    public static double launchPos = 0.4;
    public static double hold = 0.8;
    public static double release = 0.5;
    public static double hold2 = 0.8;
    public static double release2 = 0.8;

    public endGame_Mechs(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "Drone");
        this.servo.setPosition(initPos);
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


    public void release() {
        this.servo.setPosition(release);
        this.servo.setPosition(release2);
    }

    public void hold() {
        this.servo.setPosition(hold);
        this.servo.setPosition(hold2);

    }
}


