package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DRONE_ROTATION_UP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DRONE_ROTATION_UP_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    private Servo droneUpServo;

    public Drone(HardwareMap hardwareMap) {
        this.droneUpServo = hardwareMap.get(Servo.class, DRONE_ROTATION_UP_NAME);
    }

    public void raise() {
        this.droneUpServo.setPosition(0.2);

    }
}
