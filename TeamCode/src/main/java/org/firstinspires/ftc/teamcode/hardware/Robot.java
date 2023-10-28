package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

import lombok.Getter;

public class Robot {

    @Getter
    private MecanumDrive drive;

    @Getter
    private Gantry gantry;

    @Getter
    private Claw claw;

    public Robot(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.gantry = new Gantry(hardwareMap);
        this.claw = new Claw(hardwareMap);
    }
}
