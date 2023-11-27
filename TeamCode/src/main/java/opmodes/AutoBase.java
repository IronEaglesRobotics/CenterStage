package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SCORING_DISTANCE_FROM_APRIL_TAG;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Center;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Left;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Right;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Unknown;

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

@Config
public abstract class AutoBase extends LinearOpMode {
    public static int DEPOSIT_HEIGHT = 100;
    public static double SCORING_DURATION_S =  5f;
    public static double APRIL_TAG_RIGHT_DELTA = -8.5;
    public static double APRIL_TAG_LEFT_DELTA = 7.0;

    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;
    protected final Pose2d initialPosition;
    protected final CenterStageCommon.Alliance alliance;
    protected final Pose2d park;

    protected AutoBase(CenterStageCommon.Alliance alliance, Pose2d initialPosition, Pose2d park) {
        this.alliance = alliance;
        this.initialPosition = initialPosition;
        this.park = park;
    }

    @Override
    public void runOpMode() {
        // Initialize Robot and Dashboard
        this.robot = new Robot(hardwareMap, telemetry, initialPosition, alliance);
        this.dashboard = FtcDashboard.getInstance();
        this.dashboardTelemetry = dashboard.getTelemetry();

        // Wait for match to start
        while(!isStarted() && !isStopRequested()) {
            this.robot.update();
            this.sleep(20);
        }

        if (isStopRequested()) {
            return;
        }

        // If the prop is visible at this point, then it must be in the center (2) position
        determinePropLocation();

        TrajectorySequenceBuilder builder;
        switch (this.propLocation) {
            case Left:
                propLeft();
                break;
            case Unknown:
            case Center:
                propCenter();
                break;
            case Right:
                propRight();
                break;
        }

        moveToBackstage();
        prepareToScore();
        scorePreloadedPixel();
        park();
    }

    protected abstract void propLeft();

    protected abstract void propCenter();

    protected abstract void propRight();

    protected void openAndLiftClaw() {
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }

    protected void scorePreloadedPixel() {
        this.robot.getGantry().setSlideTarget(DEPOSIT_HEIGHT);
        this.robot.getGantry().armOut();
        while(this.robot.getGantry().isIn()) {
            this.robot.update();
            sleep(20);
        }
        this.robot.getGantry().deposit();
        double startTime = this.getRuntime();
        while (this.getRuntime() < (startTime + SCORING_DURATION_S)) {
            this.robot.update();
        }
        this.robot.getGantry().stop();
        this.robot.getGantry().setSlideTarget(0);
        this.robot.getGantry().armInSync();

    }

    protected void prepareToScore() {
        // At this point we know that Y = 38
        // For 2 -> Ydelta = 0
        // For 3 -> 3 5/8
        // For 1 -> - 3 5/8
        double delta = 0;
        switch (this.propLocation) {
            case Left:
                delta = APRIL_TAG_LEFT_DELTA;
                break;
            case Unknown:
            case Center:
                delta = 0;
                break;
            case Right:
                delta = APRIL_TAG_RIGHT_DELTA;
                break;
        }
        double distance = this.robot.getCamera().getDistanceToAprilTag(2, 25, 5);
        Vector2d target = new Vector2d(this.robot.getDrive().getPoseEstimate().getX() + (distance - SCORING_DISTANCE_FROM_APRIL_TAG), this.robot.getDrive().getPoseEstimate().getY() + delta);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(target);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void moveToBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(36, 11, 0));
        builder.lineToLinearHeading(new Pose2d(36, 38, 0));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void determinePropLocation() {
        setPropLocationIfVisible(Center, Unknown);
        if (this.propLocation != Center) {
            peekRight();
        }
    }

    protected void peekRight() {
        TrajectorySequenceBuilder builder = this.robot.getDrive()
                .trajectorySequenceBuilder(initialPosition);
        builder.forward(5);
        builder.turn(Math.toRadians(-33));
        this.robot.getDrive().followTrajectorySequence(builder.build());
        setPropLocationIfVisible(Right, Left);
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

    public void park() {
        double currentX = this.robot.getDrive().getPoseEstimate().getX();
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.strafeTo(new Vector2d(currentX, park.getY()));
        builder.lineToConstantHeading(park.vec());
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}
