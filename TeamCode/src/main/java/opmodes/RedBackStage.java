package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "RedBackStage", preselectTeleOp = "MainTeleOp")
public class RedBackStage extends AutoBase {

    public RedBackStage() {
        super(AutoConfig.builder()
                .initialPosition(new Pose2d(12, -63, Math.toRadians(-90)))
                .leftParkPosition(new Pose2d(61, -12))
                .rightParkPosition(new Pose2d(62, -62))
                .parkLocation(AutoConfig.ParkLocation.Left)
                .build());
    }

    @Override
    protected void propLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(18.25, -33.5, Math.toRadians(0)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(12, -42, this.config.getInitialPosition().getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(15.9, -41.35, Math.toRadians(244.15)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(new Vector2d(24, -50));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}

