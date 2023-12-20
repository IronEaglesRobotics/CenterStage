package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class DroneLauncher {
    private Servo servo;
    public static double initPos = 0;
    public static double launchPos = 0.5;

    public DroneLauncher(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "Drone Launcher");
        this.servo.setPosition(initPos);
    }

    public void launch() {
        this.servo.setPosition(launchPos);
    }

    public void reset() {
        this.servo.setPosition(initPos);
    }
}
