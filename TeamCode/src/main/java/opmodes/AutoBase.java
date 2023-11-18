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

@Config
public abstract class AutoBase extends LinearOpMode {
    public static int DEPOSIT_HEIGHT = 100;
    public static int SCORING_DURATION_MS =  5000;
    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;
    protected final Pose2d initialPosition;
    protected final CenterStageCommon.Alliance alliance;
    protected final Vector2d rendezvous;

    protected AutoBase(CenterStageCommon.Alliance alliance, Pose2d initialPosition, Vector2d rendezvous) {
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

        switch (this.propLocation) {
            case Left:
                // TODO Tommy: Place the pixel on the left tape and move to rendezvous position
                break;
            case Unknown:
            case Center:
                dislodgePropAndPlacePixel();
                break;
            case Right:
                // TODO Tommy: Place the pixel on the right tape and move to rendezvous position
                break;
        }

        moveToBackstage();
        prepareToScore();
        scorePreloadedPixel();

        // TODO Tommy: Park
    }

    private void scorePreloadedPixel() {
        this.robot.getGantry().setSlideTarget(DEPOSIT_HEIGHT);
        this.robot.getGantry().armOut();
        while(this.robot.getGantry().isIn()) {
            this.robot.getGantry().update();
            sleep(20);
        }
        this.robot.getGantry().deposit();
        this.sleep(SCORING_DURATION_MS);
        this.robot.getGantry().stop();
    }

    private void prepareToScore() {
        double distance = this.robot.getCamera().getDistanceToAprilTag(2, 25, 5);
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.forward(distance - SCORING_DISTANCE_FROM_APRIL_TAG);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    private void moveToBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(36, 11, 0));
        builder.lineToLinearHeading(new Pose2d(36, 38, 0));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    private void dislodgePropAndPlacePixel() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(rendezvous);
        builder.addDisplacementMarker(10, () -> {
            this.robot.getClaw().setArmPosition(PICKUP_ARM_MIN);
        });
        this.robot.getDrive().followTrajectorySequence(builder.build());
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }

    private void determinePropLocation() {
        setPropLocationIfVisible(Center, null);
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
