package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.FORWARD_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SCORING_DISTANCE_FROM_APRIL_TAG;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SIDE_OFFSET_IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Test", group = "Main")
public class Test extends OpMode {
    private Robot robot;

    private FtcDashboard dashboard;
    private boolean leftWasPressed;
    private boolean rightWasPressed;
    private int targetTagId;

    @Override
    public void init() {
        this.dashboard = FtcDashboard.getInstance();

        this.robot = new Robot(this.hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        robot.update();

        Vector2d poseFromAprilTag = this.robot.getCamera().getPoseFromAprilTag(2, 5);
        dashboard.getTelemetry().addData("Inferred Position", poseFromAprilTag);
        dashboard.getTelemetry().update();

        boolean leftPressed = gamepad1.dpad_left;
        boolean rightPressed = gamepad1.dpad_right;
        if (poseFromAprilTag != null) {
            if (leftPressed && !leftWasPressed) {
                macroToScore(poseFromAprilTag, true);
            } else if (rightPressed && !rightWasPressed) {
                macroToScore(poseFromAprilTag, false);
            }
        } else {
            if (leftPressed && !leftWasPressed || rightPressed && !rightWasPressed) {
                moveToStartSquare();
            }
        }

        if (!leftPressed && !rightPressed) {
            this.robot.getDrive().breakFollowing();
        }

        leftWasPressed = leftPressed;
        rightWasPressed = rightPressed;
    }

    private void moveToStartSquare() {
        if (this.robot.getDrive().isBusy()) {
            return;
        }

        Pose2d currentPoseEstimate = this.robot.getDrive().getPoseEstimate();
        if (currentPoseEstimate.getX() < 0) {
            return;
        }

        this.targetTagId = currentPoseEstimate.getY() >= 0 ? 2 : 5;

        double y = targetTagId == 2 ? 36f : -36f;

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(36, y, 0));
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
    }

    private void macroToScore(Vector2d poseFromAprilTag, boolean left) {
        Pose2d target;
        Pose2d poseEstimate = new Pose2d(poseFromAprilTag.getX(), poseFromAprilTag.getY(), this.robot.getDrive().getPoseEstimate().getHeading());
        double y = poseEstimate.getY() > 0
                ? left ? 40 : 30
                : left ? -30 : -40;
        this.robot.getDrive().setPoseEstimate(poseEstimate);
        target = new Pose2d(Camera.tag2Pose.getX() - SCORING_DISTANCE_FROM_APRIL_TAG - FORWARD_OFFSET_IN, y - SIDE_OFFSET_IN, 0);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(target);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
    }
}
