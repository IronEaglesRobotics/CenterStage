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
    public static double APRIL_TAG_LEFT_DELTA = 8.0;

    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;
    protected final Pose2d initialPosition;
    protected final CenterStageCommon.Alliance alliance;
    protected final Pose2d rendezvous;

    protected AutoBase(CenterStageCommon.Alliance alliance, Pose2d initialPosition, Pose2d rendezvous) {
        this.alliance = alliance;
        this.initialPosition = initialPosition;
        this.rendezvous = rendezvous;
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

        // If the prop is visible at this point, then it must be in the center (2) position
        determinePropLocation();

        TrajectorySequenceBuilder builder;
        switch (this.propLocation) {
            case Left:
                dislodgePropAndPlacePixelLeft();

                builder = this.robot.getTrajectorySequenceBuilder();
                builder.lineToLinearHeading(rendezvous);
                this.robot.getDrive().followTrajectorySequence(builder.build());
                break;
            case Unknown:
            case Center:
                dislodgePropAndPlacePixelCenter();

                builder = this.robot.getTrajectorySequenceBuilder();
                builder.turn(Math.toRadians(90));
                this.robot.getDrive().followTrajectorySequence(builder.build());
                break;
            case Right:
                dislodgePropAndPlacePixelRight();

                builder = this.robot.getTrajectorySequenceBuilder();
                builder.lineToLinearHeading(this.rendezvous);
                this.robot.getDrive().followTrajectorySequence(builder.build());
                break;
        }

        moveToBackstage();
        prepareToScore();
        scorePreloadedPixel();

        // TODO Tommy: Park
    }

    private void dislodgePropAndPlacePixelLeft() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-52, 31, Math.toRadians(-180)));
        builder.lineToConstantHeading(new Vector2d(-42, 31));
        builder.addTemporalMarker(0.2, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }


    private void dislodgePropAndPlacePixelRight() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(-54, 17, Math.toRadians(-123)));
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }

    private void scorePreloadedPixel() {
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

    private void prepareToScore() {
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

    private void moveToBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(36, 11, 0));
        builder.lineToLinearHeading(new Pose2d(36, 38, 0));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    private void dislodgePropAndPlacePixelCenter() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(rendezvous.vec());
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }

    private void determinePropLocation() {
        setPropLocationIfVisible(Center, Unknown);
        if (this.propLocation != Center) {
            peekRight();
        }
    }

    private void peekRight() {
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
}
