package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CAMERA_FORWARD_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SCORING_DISTANCE_FROM_APRIL_TAG;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Center;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Left;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Right;
import static org.firstinspires.ftc.teamcode.util.CenterStageCommon.PropLocation.Unknown;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;

@Config
public abstract class AutoBase extends LinearOpMode {
    protected AutoConfig config;

    protected Robot robot;

    protected AutoBase(AutoConfig config) {
        this.config = config;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // Initialize Robot and Dashboard
        this.robot = new Robot(hardwareMap, telemetry, this.config.getInitialPosition(), this.config.getAlliance());
        GamepadEx controller1 = new GamepadEx(this.gamepad1);
        GamepadEx controller2 = new GamepadEx(this.gamepad2);

        // Wait for match to start
        while (!isStarted() && !isStopRequested()) {
            this.robot.update();
            controller1.readButtons();
            controller2.readButtons();

            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)
                    || controller2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                this.config.setParkLocation(AutoConfig.ParkLocation.Left);
            }
            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)
                    || controller2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                this.config.setParkLocation(AutoConfig.ParkLocation.Right);
            }

            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_UP)
                    || controller2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                this.config.increaseDelay();
            }
            if (controller1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)
                    || controller2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                this.config.decreaseDelay();
            }

            this.telemetry.addLine("Press DPAD_UP to increase delay");
            this.telemetry.addLine("Press DPAD_DOWN to decrease delay");
            this.telemetry.addLine("Press DPAD_LEFT to park on the left");
            this.telemetry.addLine("Press DPAD_RIGHT to park on the right");
            this.telemetry.addData("Delay", String.format("%d second(s)", this.config.getDelay() / 1000));
            this.telemetry.addData("Park set to", this.config.getParkLocation());
            this.sleep(20);
        }
        if (isStopRequested()) {
            return;
        }

        CenterStageCommon.PropLocation propLocation = determinePropLocation();
        switch (propLocation) {
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

        this.sleep(this.config.getDelay());
        moveBackstage();
        prepareToScore(propLocation);
        scorePreloadedPixel();
        park();
    }

    protected abstract void propLeft();

    protected abstract void propCenter();

    protected abstract void propRight();

    protected void moveBackstage() { }

    protected void openAndLiftClaw() {
        this.robot.getClaw().openSync();
        this.sleep(100);
        this.robot.getClaw().setArmPosition(PICKUP_ARM_MAX);
    }

    protected void scorePreloadedPixel() {
        this.robot.getGantry().setSlideTarget(AutoConfig.DEPOSIT_HEIGHT);
        this.robot.getGantry().armOut();
        while (this.robot.getGantry().isIn()) {
            this.robot.update();
            sleep(20);
        }
        this.robot.getGantry().deposit();
        double startTime = this.getRuntime();
        while (this.getRuntime() < (startTime + AutoConfig.SCORING_DURATION_S)) {
            this.robot.update();
        }
        this.robot.getGantry().stop();
        this.robot.getGantry().setSlideTarget(0);
        this.robot.getGantry().armInSync();
    }

    protected void prepareToScore(CenterStageCommon.PropLocation propLocation) {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();

        this.robot.getGantry().armOut();
        this.robot.getGantry().setSlideTarget(AutoConfig.DEPOSIT_HEIGHT);

        builder.lineToLinearHeading(new Pose2d(35, Math.copySign(36, this.config.getInitialPosition().getY()), 0));

        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());

        Pose2d inferredPos;
        while (this.robot.getDrive().isBusy()) {
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

        double delta = 0;
        switch (propLocation) {
            case Left:
                delta = AutoConfig.APRIL_TAG_LEFT_DELTA;
                break;
            case Unknown:
            case Center:
                delta = 0;
                break;
            case Right:
                delta = AutoConfig.APRIL_TAG_RIGHT_DELTA;
                break;
        }

        builder = this.robot.getDrive().trajectorySequenceBuilder(inferredPos);
        Pose2d target = new Pose2d(
                60 - SCORING_DISTANCE_FROM_APRIL_TAG - CAMERA_FORWARD_OFFSET_IN, // 60 is the X position of the april tag
                Math.copySign(36, this.config.getInitialPosition().getY()) + delta,
                0);
        builder.lineToLinearHeading(target);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            this.robot.update();
        }
    }

    protected CenterStageCommon.PropLocation determinePropLocation() {
        this.robot.getClaw().setArmPositionAsync(PICKUP_ARM_MIN);

        while (!this.robot.getClaw().isArmAtPosition() && !this.robot.getCamera().getProp().isValid()) {
            this.robot.update();
            sleep(20);
        }

        sleep(250);

        CenterStageCommon.PropLocation propLocation = getPropLocationIfVisible(Center, Unknown);
        if (propLocation != Center) {
            Pose2d currentPose = this.robot.getDrive().getPoseEstimate();
            final double y = currentPose.getY() > 0 ? -5 : 5;
            final double z = Math.toRadians(-25);
            TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
            builder.lineToLinearHeading(currentPose.plus(new Pose2d(0, y, z)));
            this.robot.getDrive().followTrajectorySequence(builder.build());

            this.sleep(250);

            propLocation = getPropLocationIfVisible(Right, Left);
        }

        return propLocation;
    }

    private CenterStageCommon.PropLocation getPropLocationIfVisible(CenterStageCommon.PropLocation ifVisible, CenterStageCommon.PropLocation ifNotVisible) {
        float seenCount = 0;
        float samples = 10;
        for (int i = 0; i < samples; i++) {
            Detection detection = this.robot.getCamera().getProp();
            if (detection.isValid()) {
                seenCount++;
            }
        }
        if (seenCount / samples > 0.5) {
            return ifVisible;
        } else {
            return ifNotVisible;
        }
    }

    private void park() {
        Pose2d park = this.config.getSelectionParkPosition();
        double currentX = this.robot.getDrive().getPoseEstimate().getX();
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.strafeTo(new Vector2d(currentX, park.getY()));
        builder.lineToLinearHeading(park);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }
}
