package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "autoRed2+4")
public class AutoRedTwoPlusFour extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization = "CENTER";
    private String parkLocation = "LEFT";


    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(27, -26, Math.toRadians(360));
    final static Pose2d CENTER_PRELOAD = new Pose2d(33, -26, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(45, -27, Math.toRadians(270));
    //Board Scores
    final static Pose2d LEFT_BOARD = new Pose2d(78, -23.5, Math.toRadians(360));
    final static Pose2d CENTER_BOARD = new Pose2d(78, -33, Math.toRadians(360));
    final static Pose2d RIGHT_BOARD = new Pose2d(76, -39, Math.toRadians(360));
    //Parka
    final static Pose2d PARK = new Pose2d(60, -58, Math.toRadians(360));
    final static Pose2d PARK2 = new Pose2d(80, -60, Math.toRadians(360));
    final static Pose2d PARKLEFT = new Pose2d(50, -15, Math.toRadians(360));
    final static Pose2d PARKLEFT2 = new Pose2d(75, -12, Math.toRadians(360));
    //Cycles
    final static Vector2d LEAVE_BOARD = new Vector2d(50, -10);
    final static Vector2d TO_STACK = new Vector2d(-35, -10);
    final static Vector2d TO_STACK_SLOW = new Vector2d(-47.5, -10);
    final static Pose2d TO_STACK_SLOW2 = new Pose2d(-38.5, -8, Math.toRadians(360));
    final static Pose2d BACK_THROUGH_GATE = new Pose2d(50, -10, Math.toRadians(360));
    final static Pose2d APPROACHING_BOARD = new Pose2d(70, -31, Math.toRadians(360));
    final static Vector2d SCORE_STACK = new Vector2d(72.5, -35);
    final static Vector2d SCORE_STACK_SLOW = new Vector2d(73, -35);
    //Park
    final static Pose2d BACK_OFF = new Pose2d(60, -58, Math.toRadians(360));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.setReversed(true);
                builder.splineToSplineHeading(LEFT_PRELOAD_TWO, Math.toRadians(180),
                        MecanumDrive.getVelocityConstraint(70, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(50));
                builder.setReversed(false);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD);
                break;
        }
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void boardScore() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_BOARD);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_BOARD,
                        MecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(50));
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD,
                        MecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(30));
                break;
        }
        builder.addTemporalMarker(.2, robot.getArm()::armSecondaryScore);
        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void toStack() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.setReversed(true);
        builder.splineToConstantHeading(LEAVE_BOARD, Math.toRadians(180));
        builder.lineToConstantHeading(TO_STACK,
                MecanumDrive.getVelocityConstraint(80, Math.toRadians(360), DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(80));
        builder.setReversed(false);
        switch (randomization) {
            case "LEFT":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, 1)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
            case "CENTER":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, 3.2)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
            case "RIGHT":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, 0.25)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
        }
        builder.addTemporalMarker(.3, robot.getArm()::armPickupStack);
        builder.addTemporalMarker(.3, robot.getClaw()::close);
        builder.addTemporalMarker(.3, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(2, robot.getClaw()::openStack);
        builder.addTemporalMarker(.2, robot.getSlides()::slideDown);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void toStackLower() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.setReversed(true);
        builder.splineToConstantHeading(LEAVE_BOARD.plus(new Vector2d(0, -1)), Math.toRadians(180));
        builder.lineToConstantHeading(TO_STACK.plus(new Vector2d(0, -1)),
                MecanumDrive.getVelocityConstraint(80, Math.toRadians(360), DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(80));
        builder.setReversed(false);
        switch (randomization) {
            case "LEFT":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, 0)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
            case "CENTER":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, -1)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
            case "RIGHT":
                builder.lineToConstantHeading(TO_STACK_SLOW.plus(new Vector2d(0, -3)),
                        MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(40));
                break;
        }
        builder.addTemporalMarker(.3, robot.getArm()::armPickupStackLow);
        builder.addTemporalMarker(.3, robot.getClaw()::close);
        builder.addTemporalMarker(.3, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.7, robot.getClaw()::openStack);
        builder.addTemporalMarker(.2, robot.getSlides()::slideDown);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void toBoard() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(LEAVE_BOARD);
        builder.splineToConstantHeading(SCORE_STACK, Math.toRadians(360),
                MecanumDrive.getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(40));
        builder.addTemporalMarker(.01, robot.getArm()::armPickdaUpy);
        builder.addTemporalMarker(2.2, robot.getArm()::armScoreStack);
        builder.addTemporalMarker(2.4, robot.getWrist()::wristScore);
        builder.addTemporalMarker(2.5, robot.getSlides()::slideFirstLayer);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void toStage() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(PARKLEFT2.vec());
        builder.addTemporalMarker(.01, robot.getArm()::armPickdaUpy);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void scoreTest() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToConstantHeading(SCORE_STACK_SLOW,
                MecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(30));
        builder.addTemporalMarker(.3, this::clawSlowOpen);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void clawSlowOpen() {
        double currentPos = .62;
        double targetPos = .6;
        double delta = (targetPos - currentPos) / 100;
        for (int i = 0; i < 100; i++) {
            this.robot.getClaw().setPos(currentPos + (delta * i));
            this.robot.getSlides().slideFirstLayer();
            sleep(5);
        }
    }

    protected void park() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (parkLocation) {
            case "LEFT":
                builder.lineToLinearHeading(PARKLEFT);
                builder.lineToLinearHeading(PARKLEFT2);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(PARK);
                builder.lineToLinearHeading(PARK2);
                break;
        }
        builder.addTemporalMarker(.1, robot.getArm()::armRest);
        builder.addTemporalMarker(.1, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void parkLocation() {
        if (gamepad2.dpad_left) {
            parkLocation = "LEFT";
        } else if (gamepad2.dpad_right) {
            parkLocation = "RIGHT";
        }
    }

    protected void startLocation() {
        if (gamepad2.x) {
            randomization = "LEFT";
        } else if (gamepad2.y) {
            randomization = "CENTER";
        } else if (gamepad2.b) {
            randomization = "RIGHT";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
//        this.robot.getCamera().setAlliance(Alliance.Blue);
//        this.robot.getCamera().initTargetingCamera();
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
//            randomization = String.valueOf(this.robot.getCamera().getStartingPositionBlue());
            parkLocation();
            startLocation();
            this.telemetry.addData("Starting Position", randomization);
            this.telemetry.addData("Park Position", parkLocation);
            this.telemetry.update();
        }
        robot.update();
        this.robot.getClaw().close();
        scorePreloadOne();
        boardScore();
//        sleep(250);
        this.robot.getClaw().open();
        sleep(250);
        toStack();
        this.robot.getClaw().close();
        sleep(300);
        toBoard();
        clawSlowOpen();
//            sleep(100);
        toStackLower();
        this.robot.getClaw().close();
        sleep(300);
        switch (randomization) {
            default:
                toBoard();
                clawSlowOpen();
                break;
//                case "LEFT":
//                    toStage();
//                    break;
        }
        while (!isStopRequested()) {
            this.robot.getArm().armRest(true);
            this.robot.getWrist().wristPickup();
            this.robot.getSlides().slideDown();
            this.robot.getClaw().open();
            robot.update();
        }
//            park();
    }

}

