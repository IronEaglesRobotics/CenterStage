package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "BlueBackStage", preselectTeleOp = "MainTeleOp")
public class BlueBackStage extends AutoBase {
    private static final int delay = 0;

    public BlueBackStage() {
        super(
                CenterStageCommon.Alliance.Blue,
                new Pose2d(12, 63, Math.toRadians(90)),
                new Pose2d(62, 62));
    }

    protected void propLeft() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(17.3, 41.6, Math.toRadians(108.25)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(new Vector2d(24, 50));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void propCenter() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(12, 42, initialPosition.getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propRight() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(18.25, 32, Math.toRadians(0)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }
}
