package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "BlueFrontStage", preselectTeleOp = "MainTeleOp")
public class BlueFrontStage extends AutoBase {

    public BlueFrontStage() {
        super(AutoConfig.builder()
                .initialPosition(new Pose2d(-36, 63, Math.toRadians(90)))
                .leftParkPosition(new Pose2d(62, 62))
                .rightParkPosition(new Pose2d(62, 12, Math.toRadians(0)))
                .parkLocation(AutoConfig.ParkLocation.Left)
                .build());
    }

    @Override
    protected void propLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-43.5, 31.5, Math.toRadians(180)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-36, 42, this.config.getInitialPosition().getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-39.34, 40.45, Math.toRadians(64.3)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void moveBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToSplineHeading(new Pose2d(-40, 60, Math.PI));
        builder.lineToLinearHeading(new Pose2d(12, 60, Math.PI));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}