package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SCORING_DISTANCE_FROM_APRIL_TAG;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Center;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Left;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Right;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Point;

@Config
public abstract class AutoBase extends LinearOpMode {
    public static int DEPOSIT_HEIGHT = 100;

    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
        this.dashboard = FtcDashboard.getInstance();
        this.dashboardTelemetry = dashboard.getTelemetry();

        this.robot.getCamera().setAlliance(CenterStageCommon.Alliance.Blue);

        this.robot.getDrive().setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(-90)));

        while(!isStarted() && !isStopRequested()) {
            this.robot.update();
            this.sleep(20);
        }

        setPropLocationIfVisible(Center, null);

        TrajectorySequenceBuilder builder = this.robot.getDrive()
                .trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(-90)));
        if (this.propLocation != CenterStageCommon.PropLocation.Center) {
            builder.forward(5);
            builder.turn(Math.toRadians(-33));
            this.robot.getDrive().followTrajectorySequence(builder.build());

            setPropLocationIfVisible(Right, Left);
            return;
        } else {
            // Center
            builder.lineToConstantHeading(new Vector2d(-36, 11));
            builder.addDisplacementMarker(10, () -> {
                this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
            });
            this.robot.getDrive().followTrajectorySequence(builder.build());

            this.robot.getClaw().openSync();
            this.sleep(100);
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
        }

        builder = this.robot.getDrive().trajectorySequenceBuilder(new Pose2d(-36, 11));
        builder.lineToLinearHeading(new Pose2d(36, 11, 0));
        builder.lineToLinearHeading(new Pose2d(36, 38, 0));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        double distance = getDistanceToAprilTag();

        builder = this.robot.getDrive().trajectorySequenceBuilder(new Pose2d(36, 38, 0));
        builder.forward(distance - SCORING_DISTANCE_FROM_APRIL_TAG);
        this.robot.getDrive().followTrajectorySequence(builder.build());

        this.robot.getGantry().setSlideTarget(DEPOSIT_HEIGHT);
        this.robot.getGantry().armOut();
        while(this.robot.getGantry().isIn()) {
            this.robot.getGantry().update();
            sleep(20);
        }
        this.robot.getGantry().deposit();

        while(opModeIsActive()) {
            this.robot.update();
            AprilTagDetection aprilTagDetection = this.robot.getCamera().getAprilTag(2);
            if (aprilTagDetection != null) {
                Point center = aprilTagDetection.center;
                this.dashboardTelemetry.addData("center", center);
                this.dashboardTelemetry.addData("x", aprilTagDetection.ftcPose.x);
                this.dashboardTelemetry.addData("y", aprilTagDetection.ftcPose.y);
                this.dashboardTelemetry.addData("z", aprilTagDetection.ftcPose.z);
                this.dashboardTelemetry.update();
            }
            sleep(20);
        }
    }

    private double getDistanceToAprilTag() {
        double minDistance = Double.MAX_VALUE;
        for (int i = 0; i < 10; i++) {
            AprilTagDetection aprilTagDetection = this.robot.getCamera().getAprilTag(2);
            if (aprilTagDetection != null) {
                if (aprilTagDetection.ftcPose.y < minDistance) {
                    minDistance = aprilTagDetection.ftcPose.y;
                }
            }
        }
        return minDistance;
    }

    protected static int getExpectedAprilTagId(CenterStageCommon.PropLocation propLocation) {
        switch (propLocation) {
            case Left:
                return 1;
            case Unknown:
            case Center:
                return 2;
            case Right:
                return 3;
        }

        return 2;
    }

    protected void setPropLocationIfVisible(CenterStageCommon.PropLocation ifVisible, CenterStageCommon.PropLocation ifNotVisible) {
        float seenCount = 0;
        float samples = 10;
        for (int i = 0; i < samples; i++) {
            Detection detection = this.robot.getCamera().getProp();
            if (detection.isValid()) {
                seenCount++;
            }
        }
        if (seenCount / samples > 0.5) {
            this.propLocation = ifVisible;
        } else {
            this.propLocation = ifNotVisible;
        }
    }
}
