package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "BlueFrontStage", preselectTeleOp = "MainTeleOp")
public class BlueFrontStage extends AutoBase {

    public BlueFrontStage() {
        super(
                CenterStageCommon.Alliance.Blue,
                new Pose2d(-36, 63, Math.toRadians(90)),
                new Pose2d(62, 62),
                new Pose2d(62, 12, Math.toRadians(0)));

    }

    protected void propLeft() {

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-43.5, 31.5, Math.toRadians(180)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propCenter() {

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-36, 42, initialPosition.getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propRight() {

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-39.34, 40.45, Math.toRadians(64.3)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }
}