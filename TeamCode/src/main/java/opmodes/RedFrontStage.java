package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "RedFrontStage", preselectTeleOp = "MainTeleOp")
public class RedFrontStage extends AutoBase {
    private static final int delay = 0;

    public RedFrontStage() {
        super(
                CenterStageCommon.Alliance.Red,
                new Pose2d(-36, -63, Math.toRadians(-90)),
                new Pose2d(61, -12));
    }

    protected void propLeft() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-40.46, -41.93, Math.toRadians(291.8)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propCenter() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-36, -42, initialPosition.getHeading()));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }

    protected void propRight() {
        this.sleep(delay);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-41.82, -33.68, Math.toRadians(180)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();
    }
}
