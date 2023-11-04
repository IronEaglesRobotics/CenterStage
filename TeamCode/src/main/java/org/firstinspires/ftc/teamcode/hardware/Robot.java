package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

import lombok.Getter;

public class Robot {

//    @Getter
//    private MecanumDrive drive;

    @Getter
    private Drive drive;

    @Getter
    private Gantry gantry;

    @Getter
    private Claw claw;

    @Getter
    private RobotLift lift;

    public Robot(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
//        this.drive = new MecanumDrive(hardwareMap);
        this.drive = new Drive(hardwareMap);
//        this.gantry = new Gantry(hardwareMap);
        this.claw = new Claw(hardwareMap);
//        this.lift = new RobotLift(hardwareMap);
    }
}
