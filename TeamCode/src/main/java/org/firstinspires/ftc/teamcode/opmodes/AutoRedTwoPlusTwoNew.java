package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
@Config
@Autonomous(name = "autoRed2+2New")
public class AutoRedTwoPlusTwoNew extends LinearOpMode {
    protected Pose2d initialPosition;

    protected Trajectory park1;
    protected Trajectory park2;

    private Robot robot;
    private String randomization;

    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_ONE = new Pose2d(40, -37.5, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(29.5, -29, Math.toRadians(360));
    final static Pose2d CENTER_PRELOAD = new Pose2d(33, -28, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(46, -35, Math.toRadians(270));
    //Board Scores
    final static Pose2d LEFT_BOARD = new Pose2d(76, -26.5, Math.toRadians(360));
    final static Pose2d CENTER_BOARD = new Pose2d(75.7, -36.3, Math.toRadians(360));
    final static Pose2d RIGHT_BOARD = new Pose2d(75, -42, Math.toRadians(360));
    //Stack Cycle
    final static Pose2d LEAVE_BOARD = new Pose2d(65, -10, Math.toRadians(360));
    final static Pose2d TO_STACK = new Pose2d(-41, -8, Math.toRadians(360));
    final static Pose2d BACK_THROUGH_GATE = new Pose2d(50, -10, Math.toRadians(360));
    final static Pose2d APPROACHING_BOARD = new Pose2d(73.5, -28, Math.toRadians(360));
    final static Pose2d SCORE_STACK = new Pose2d(72.5, -28, Math.toRadians(360));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD_ONE);
                builder.lineToLinearHeading(LEFT_PRELOAD_TWO);
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD);
        }
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void boardScore() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_BOARD);
            case "CENTER":
                builder.lineToLinearHeading(CENTER_BOARD);
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD);
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
        builder.lineToLinearHeading(TO_STACK);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void scoreStack()  {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(BACK_THROUGH_GATE);
        builder.lineToLinearHeading(APPROACHING_BOARD);
        builder.lineToLinearHeading(SCORE_STACK);
        builder.addTemporalMarker(2, robot.getArm()::armSecondaryScore);
        builder.addTemporalMarker(2, robot.getWrist()::wristScore);
        builder.addTemporalMarker(2, robot.getSlides()::slideAutoStacks);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void clawSlowOpen() {
        double currentPos = .86;
        double targetPos = .78;
        double delta = (targetPos - currentPos) / 100;
        for (int i = 0; i < 100; i++) {
            this.robot.getClaw().setPos(currentPos + (delta * i));
            sleep(12);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.robot.getCamera().initTargetingCamera();
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
                .lineToLinearHeading(new Pose2d(80, -57, Math.toRadians(360)))
                .build();

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPosition());
            randomization = String.valueOf(this.robot.getCamera().getStartingPosition());
            this.telemetry.update();
        }

        scorePreloadOne();
        boardScore();

        sleep(250);
        this.robot.getClaw().open();
        sleep(250);

        toStack();

        sleep(500);
        this.robot.getClaw().setPos(.86);
        sleep(250);
        scoreStack();

        sleep(200);
        clawSlowOpen();
        sleep(300);
    }

}
