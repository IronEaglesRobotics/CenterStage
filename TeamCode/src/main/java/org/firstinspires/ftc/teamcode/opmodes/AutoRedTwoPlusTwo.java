package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "autoRed2+2")
public class AutoRedTwoPlusTwo extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization;

    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_ONE = new Pose2d(40, -37.5, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(29.5, -32, Math.toRadians(360));
    final static Pose2d CENTER_PRELOAD = new Pose2d(34, -28, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(43, -35, Math.toRadians(270));
    //Board Scores
    final static Pose2d LEFT_BOARD = new Pose2d(76.7, -27, Math.toRadians(360));
    final static Pose2d CENTER_BOARD = new Pose2d(76.5, -32, Math.toRadians(355));
    final static Pose2d RIGHT_BOARD = new Pose2d(76.5, -40, Math.toRadians(355));
    //Stack Cycle
    final static Pose2d LEAVE_BOARD = new Pose2d(65, -10, Math.toRadians(360));
    final static Pose2d TO_STACK = new Pose2d(-35, -8, Math.toRadians(360));
    final static Pose2d TO_STACK_SLOW = new Pose2d(-38.5, -8.5, Math.toRadians(360));
    final static Pose2d TO_STACK_SLOW2 = new Pose2d(-38.5, -8, Math.toRadians(360));
    final static Pose2d BACK_THROUGH_GATE = new Pose2d(50, -10, Math.toRadians(360));
    final static Pose2d APPROACHING_BOARD = new Pose2d(70, -31, Math.toRadians(360));
    final static Pose2d SCORE_STACK = new Pose2d(73.5, -31, Math.toRadians(360));
    //Park
    final static Pose2d BACK_OFF = new Pose2d(60, -58, Math.toRadians(360));
    final static Pose2d PARK = new Pose2d(80, -60, Math.toRadians(360));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD_ONE);
                builder.lineToLinearHeading(LEFT_PRELOAD_TWO);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD);
                break;
        }
        builder.addTemporalMarker(.5, robot.getArm()::armScore);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void boardScore() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_BOARD,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                ;
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_BOARD,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                break;
        }
        builder.addTemporalMarker(.2, robot.getArm()::armScore);
        builder.addTemporalMarker(.2, robot.getSlides()::slideFirstLayer);
        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void toStack() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(LEAVE_BOARD);
        builder.addTemporalMarker(.3, robot.getArm()::armRest);
        builder.addTemporalMarker(.3, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
        builder.addTemporalMarker(1.5, robot.getClaw()::openStack);
        builder.addTemporalMarker(1.5, robot.getArm()::pickup);
        builder.lineToLinearHeading(TO_STACK);
        builder.lineToLinearHeading(TO_STACK_SLOW,
                MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(20));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void toStackNoDrift() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(LEAVE_BOARD);
        builder.addTemporalMarker(.3, robot.getArm()::armRest);
        builder.addTemporalMarker(.3, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
        builder.addTemporalMarker(1.5, robot.getClaw()::openStack);
        builder.addTemporalMarker(1.5, robot.getArm()::pickup);
        builder.lineToLinearHeading(TO_STACK);
        builder.lineToLinearHeading(TO_STACK_SLOW2,
                MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(20));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void scoreStack() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(BACK_THROUGH_GATE);
        builder.lineToLinearHeading(APPROACHING_BOARD);
        builder.lineToLinearHeading(SCORE_STACK);
        builder.addTemporalMarker(2.5, robot.getArm()::armSecondaryScore);
        builder.addTemporalMarker(2.5, robot.getWrist()::wristScore);
        builder.addTemporalMarker(2.5, robot.getSlides()::slideScoreStack);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void scoreTest() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(new Pose2d(75, -31, Math.toRadians(360)),
                MecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                MecanumDrive.getAccelerationConstraint(20));
        builder.addTemporalMarker(.2, this::clawSlowOpen);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void clawSlowOpen() {
        double currentPos = .8;
        double targetPos = .7;
        double delta = (targetPos - currentPos) / 100;
        for (int i = 0; i < 100; i++) {
            this.robot.getClaw().setPos(currentPos + (delta * i));
            sleep(30);
        }
    }

    protected void park() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(BACK_OFF);
        builder.lineToLinearHeading(PARK);
        builder.addTemporalMarker(.1, robot.getArm()::armRest);
        builder.addTemporalMarker(.1, robot.getClaw()::close);
        builder.addTemporalMarker(.1, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPosition());
            randomization = String.valueOf(this.robot.getCamera().getStartingPosition());
            this.telemetry.update();
        }
        this.robot.update();
        scorePreloadOne();
        boardScore();

        sleep(150);
        this.robot.getClaw().open();
        sleep(150);

        switch (randomization) {
            case "LEFT":
                toStackNoDrift();
                break;
            case "CENTER":
                toStack();
                break;
            case "RIGHT":
                toStack();
                break;
        }
        sleep(500);
        this.robot.getClaw().close();
        sleep(250);
        this.robot.getArm().armRest();
        scoreStack();
        this.robot.refreshPoseEstimateFromAprilTag();
        this.robot.getClaw().setPos(.83);
        scoreTest();
        park();
    }

}
