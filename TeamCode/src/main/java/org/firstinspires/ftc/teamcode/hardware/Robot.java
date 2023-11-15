package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
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

    @Getter
    private Camera camera;

    private final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.gantry = new Gantry(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap, telemetry);
        this.lift = new RobotLift(hardwareMap, telemetry);
        this.camera = new Camera(hardwareMap, telemetry);
    }

    public void update() {
        this.gantry.update();
        this.lift.update();
        this.drive.update();
        this.claw.update();

        this.telemetry.update();
    }
}
