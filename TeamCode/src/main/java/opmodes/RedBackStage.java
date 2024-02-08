package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "RedBackStage", preselectTeleOp = "MainTeleOp")
public class RedBackStage extends AutoBase {

    public RedBackStage() {
        super(
                CenterStageCommon.Alliance.Red,
                new Pose2d(12, -63, Math.toRadians(-90)),
                new Pose2d(61, -12),
                new Pose2d(62, -62));
    }

    protected void propLeft() {

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(18.25, -33.5, Math.toRadians(0)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propCenter() {

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(12, -42, initialPosition.getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

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

