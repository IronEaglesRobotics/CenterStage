package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "RedFrontStage", preselectTeleOp = "MainTeleOp")
public class RedFrontStage extends AutoBase {

    public RedFrontStage() {
        super(AutoConfig.builder()
                .initialPosition(new Pose2d(-36, -63, Math.toRadians(-90)))
                .leftParkPosition(new Pose2d(61, -12))
                .rightParkPosition(new Pose2d(62, -62))
                .parkLocation(AutoConfig.ParkLocation.Right)
                .build());
    }

    @Override
    protected void propLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-40.46, -41.93, Math.toRadians(291.8)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-36, -42, this.config.getInitialPosition().getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-41.82, -33.68, Math.toRadians(180)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void moveBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToSplineHeading(new Pose2d(-40, -60, Math.PI));
        builder.lineToLinearHeading(new Pose2d(12, -60, Math.PI));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}
