package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private Drive drive;

    public Robot(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new Drive(hardwareMap);
    }

    public Drive getDrive() {
        return this.drive;
    }
}
