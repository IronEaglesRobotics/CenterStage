package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

import lombok.Getter;

public class Robot {

    @Getter
    private MecanumDrive drive;

    @Getter
    private Gantry gantry;

    @Getter
    private Claw claw;

    @Getter
    private RobotLift lift;

    private Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.gantry = new Gantry(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap);
        this.lift = new RobotLift(hardwareMap, telemetry);
    }

    public void update() {
        this.gantry.update();
        this.lift.update();
        this.telemetry.update();
        this.drive.update();
        this.claw.update();

//        Pose2d pose = this.drive.getLocalizer().getPoseEstimate();
//        this.telemetry.addData("x", pose.getX());
//        this.telemetry.addData("y", pose.getY());
    }
}
