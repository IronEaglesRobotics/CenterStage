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
    public static double SCORING_DURATION_S =  2f; // for spin of axle
    public static double APRIL_TAG_RIGHT_DELTA = -8.5;
    public static double APRIL_TAG_LEFT_DELTA = 7.0;
    protected static double Delay = 5000;

    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;
    protected final Pose2d initialPosition;
    protected final CenterStageCommon.Alliance alliance;
    protected final Pose2d parkLeft;
    protected final Pose2d parkRight;

    protected Pose2d park;
    protected int delay = 0;
    boolean leftWasPressed = false;
    boolean rightWasPressed = false;
    boolean upWasPressed = false;
    boolean downWasPressed = false;




    protected AutoBase(CenterStageCommon.Alliance alliance, Pose2d initialPosition, Pose2d parkLeft, Pose2d parkRight) {
        this.alliance = alliance;
        this.initialPosition = initialPosition;
        this.parkLeft = parkLeft;
        this.parkRight = parkRight;
    }

    @Override
    public void runOpMode() {
        // Initialize Robot and Dashboard
        this.robot = new Robot(hardwareMap, telemetry, initialPosition, alliance);
        this.dashboard = FtcDashboard.getInstance();
        this.dashboardTelemetry = dashboard.getTelemetry();
        this.park = parkLeft;

        // Wait for match to start
        while(!isStarted() && !isStopRequested()) {
            this.robot.update();

            boolean leftPressed = gamepad1.dpad_left;
            boolean rightPressed = gamepad1.dpad_right;
            boolean upPressed = gamepad1.dpad_up;
            boolean downPressed = gamepad1.dpad_down;
            this.telemetry.addData("To select parking location, use the dpad right or left. To add delay, use the dpad up to increase delay, and dpad down to decrease delay", "");
            if(leftPressed && !leftWasPressed) {
                this.park = parkLeft;
            } else if(rightPressed && !rightWasPressed) {
                this.park = parkRight;
            } else if(upPressed && !upWasPressed) {
                this.delay += 1000;
            } else if(downPressed && !downWasPressed) {
                this.delay -= 1000;
            }

            this.leftWasPressed = leftPressed;
            this.rightWasPressed = rightPressed;
            this.upWasPressed = upPressed;
            this.downWasPressed = downPressed;

            this.telemetry.addData("Delay", this.delay);
            this.telemetry.addData("Park set to", this.park);
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
        this.sleep(delay);
        moveBackstage();
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
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();

        this.robot.getGantry().armOut();
        this.robot.getGantry().setSlideTarget(DEPOSIT_HEIGHT);

        if (this.alliance == CenterStageCommon.Alliance.Blue) {
            builder.lineToLinearHeading(new Pose2d(35, 36, 0));
        } else if (this.alliance == CenterStageCommon.Alliance.Red) {
            builder.lineToLinearHeading(new Pose2d(35, -35, 0));
        }

        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());

        Pose2d inferredPos = null;
        while(this.robot.getDrive().isBusy()) {
            this.robot.update();

            inferredPos = this.robot.getCamera().estimatePoseFromAprilTag();
            if (inferredPos != null) {
                this.robot.getDrive().breakFollowing();
                break;
            }
        }
        sleep(200);
        inferredPos = this.robot.getCamera().estimatePoseFromAprilTag();
        if (inferredPos == null) {
            inferredPos = this.robot.getDrive().getPoseEstimate();
        }

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

        builder = this.robot.getDrive().trajectorySequenceBuilder(inferredPos);
        Pose2d target = new Pose2d(
                60 - SCORING_DISTANCE_FROM_APRIL_TAG - CAMERA_FORWARD_OFFSET_IN, // 60 is the X position of the april tag
                36 + delta,
                0);
        builder.lineToLinearHeading(target);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while(this.robot.getDrive().isBusy()) {
            this.robot.update();
        }
    }

    protected void moveBackstage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();

        if (!this.isBackstage()) {
            if (this.alliance == CenterStageCommon.Alliance.Blue) {
                builder.lineToSplineHeading(new Pose2d(-40, 60, Math.PI));
                builder.lineToLinearHeading(new Pose2d(12, 60, Math.PI));
            } else if (this.alliance == CenterStageCommon.Alliance.Red) {
                builder.lineToSplineHeading(new Pose2d(-40, -60, Math.PI));
                builder.lineToLinearHeading(new Pose2d(12, -60, Math.PI));
            }
            this.robot.getDrive().followTrajectorySequence(builder.build());
        }
    }

    protected void determinePropLocation() {
        this.robot.getClaw().setArmPositionAsync(PICKUP_ARM_MIN); // changed from setArmPositionAsync

        while (!this.robot.getClaw().isArmAtPosition() && !this.robot.getCamera().getProp().isValid()) {
            this.robot.update();
            sleep(20);
        }

        sleep(250);

        setPropLocationIfVisible(Center, Unknown); //only works if arm is async
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
