package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DRONE_LAUNCH_POS;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DRONE_ROTATION_UP_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DRONE_STOW_POS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    private Servo droneLaunchServo;

    public Drone(HardwareMap hardwareMap) {
        this.droneLaunchServo = hardwareMap.get(Servo.class, DRONE_ROTATION_UP_NAME);
    }

    public void raise() {
        this.droneLaunchServo.setPosition(DRONE_LAUNCH_POS);
    }

    public void reset() {
        this.droneLaunchServo.setPosition(DRONE_STOW_POS);
    }

}
