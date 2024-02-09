package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "BlueBackStage", preselectTeleOp = "MainTeleOp")
public class BlueBackStage extends AutoBase {

    public BlueBackStage() {
        super(AutoConfig.builder()
                .initialPosition(new Pose2d(12, 63, Math.toRadians(90)))
                .leftParkPosition(new Pose2d(62, 62))
                .rightParkPosition(new Pose2d(62, 12, Math.toRadians(0)))
                .parkLocation(AutoConfig.ParkLocation.Right)
                .build());
    }

    @Override
    protected void propLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(17.3, 41.6, Math.toRadians(108.25)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(new Vector2d(24, 50));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    @Override
    protected void propCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(12, 42, this.config.getInitialPosition().getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    @Override
    protected void propRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(18.25, 32, Math.toRadians(0)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }
}
