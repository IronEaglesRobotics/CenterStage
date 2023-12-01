package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "RedBackStage", preselectTeleOp = "MainTeleOp")
public class RedBackStage extends AutoBase {
    private final Pose2d rendezvous = new Pose2d(12, 11);

    public RedBackStage() {
        super(
                CenterStageCommon.Alliance.Red,
                new Pose2d(12, 63, Math.toRadians(-90)),
                new Pose2d(62, -62));
    }

    protected void propLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(32, 34, Math.toRadians(0)));
        builder.lineToConstantHeading(new Vector2d(19, 34));
        builder.addTemporalMarker(0.5, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(rendezvous.vec());
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.turn(Math.toRadians(-90));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void propRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(36, 25, Math.toRadians(-33)));
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }
}

