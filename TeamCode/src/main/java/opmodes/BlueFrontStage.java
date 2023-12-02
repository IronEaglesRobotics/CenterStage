package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "BlueFrontStage", preselectTeleOp = "MainTeleOp")
public class BlueFrontStage extends AutoBase {
    private final Pose2d rendezvous = new Pose2d(-36, 11);

    public BlueFrontStage() {
        super(
                CenterStageCommon.Alliance.Blue,
                new Pose2d(-36, 63, Math.toRadians(-90)),
                new Pose2d(62, 12));
    }

    protected void propLeft() {
        this.sleep(5000);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-52, 31, Math.toRadians(-180)));
        builder.lineToConstantHeading(new Vector2d(-42, 31));
        builder.addTemporalMarker(0.2, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(rendezvous);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
    protected void propCenter() {
        this.sleep(5000);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(rendezvous.vec());
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.turn(Math.toRadians(90));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
    protected void propRight() {
        this.sleep(5000);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-54, 17, Math.toRadians(-123)));
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());

        openAndLiftClaw();

        builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(this.rendezvous);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}