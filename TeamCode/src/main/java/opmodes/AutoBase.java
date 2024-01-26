package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CAMERA_FORWARD_OFFSET_IN;
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
    public static double SCORING_DURATION_S =  3f; // for spin of axle
    public static double APRIL_TAG_RIGHT_DELTA = -8.5;
    public static double APRIL_TAG_LEFT_DELTA = 7.0;
    protected static double Delay = 5000;

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
        if (isStopRequested()) {  // remove later if nessacary as recent addition might be interefering
            return;
        }

        // If the prop is visible at this point, then it must be in the center (2) position
        determinePropLocation();

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
        moveToEasel();
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

        Pose2d inferredPos = this.robot.getCamera().estimatePoseFromAprilTag();
        this.robot.getDrive().setPoseEstimate(inferredPos);
        Pose2d target = new Pose2d(
                60 - SCORING_DISTANCE_FROM_APRIL_TAG - CAMERA_FORWARD_OFFSET_IN, // 60 is the X position of the april tag
                inferredPos.getY() + delta,
                0);

        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(target);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void moveToEasel() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();

        if (!this.isBackstage()) {
            if (this.alliance == CenterStageCommon.Alliance.Blue) {
                builder.lineToSplineHeading(new Pose2d(-40, 60, Math.PI));
                builder.lineToLinearHeading(new Pose2d(12, 60, Math.PI));
            } else if (this.alliance == CenterStageCommon.Alliance.Red) {
                builder.lineToSplineHeading(new Pose2d(-40, -60, Math.PI));
                builder.lineToLinearHeading(new Pose2d(12, -60, Math.PI));
            }
        }

        if (this.alliance == CenterStageCommon.Alliance.Blue) {
            builder.lineToLinearHeading(new Pose2d(35, 36, 0));
        } else if (this.alliance == CenterStageCommon.Alliance.Red) {
            builder.lineToLinearHeading(new Pose2d(35, -35, 0));
        }

        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void determinePropLocation() {
        this.robot.getClaw().setArmPositionAsync(PICKUP_ARM_MIN);

        while (!this.robot.getClaw().isArmAtPosition()) {
            this.robot.update();
            sleep(20);
        }

        sleep(250);

        setPropLocationIfVisible(Center, Unknown);
        if (this.propLocation != Center) {
            peekRight();
        }
    }

    protected void peekRight() {
        Pose2d currentPose = this.robot.getDrive().getPoseEstimate();
        final double y = currentPose.getY() > 0 ? -5 : 5;
        final double z = Math.toRadians(-25);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(currentPose.plus(new Pose2d(0, y, z)));
        this.robot.getDrive().followTrajectorySequence(builder.build());

        this.sleep(250);

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
        builder.lineToLinearHeading(park);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected boolean isBackstage() {
        return this.initialPosition.getX() > 0;
    }
}
