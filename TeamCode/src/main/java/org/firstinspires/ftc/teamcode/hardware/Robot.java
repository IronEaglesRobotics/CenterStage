package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import lombok.Getter;

public class Robot {

    @Getter
    private Drive drive;

    public Robot(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new Drive(hardwareMap);
    }
}
