package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

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

    @Getter
    CenterStageCommon.Alliance alliance;

    private final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPosition, CenterStageCommon.Alliance alliance) {
        this(hardwareMap, telemetry);
        this.getDrive().setPoseEstimate(initialPosition);
        this.setAlliance(alliance);
    }

    private void init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.gantry = new Gantry(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap, telemetry);
        this.lift = new RobotLift(hardwareMap, telemetry);
        this.camera = new Camera(hardwareMap, telemetry);
    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        this.drive.update();
        return this.drive.trajectorySequenceBuilder(this.drive.getPoseEstimate());
    }

    public void setAlliance(CenterStageCommon.Alliance alliance) {
        this.alliance = alliance;
        this.camera.setAlliance(alliance);
    }

    public void update() {
        this.gantry.update();
        this.lift.update();
        this.drive.update();
        this.claw.update();

        this.telemetry.update();
    }
}
