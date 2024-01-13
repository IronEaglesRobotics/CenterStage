package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name = "autoBlue")
public class AutoBlue extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization;

    //Pose2ds
    //Preloads
    final static Pose2d RIGHT_PRELOAD_ONE = new Pose2d(-40, -37.5, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD_TWO = new Pose2d(-29.5, -29, Math.toRadians(180));
    final static Pose2d CENTER_PRELOAD = new Pose2d(-33, -28, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD = new Pose2d(-43, -35, Math.toRadians(270));
    //Board Scores
    final static Pose2d RIGHT_BOARD = new Pose2d(-75.3, -24.5, Math.toRadians(185));
    final static Pose2d CENTER_BOARD = new Pose2d(-75.3, -35, Math.toRadians(185));
    final static Pose2d LEFT_BOARD = new Pose2d(-75.3, -42, Math.toRadians(185));
    //Park
    final static Pose2d BACK_OFF =  new Pose2d(-60,-58,Math.toRadians(180));
    final static Pose2d PARK = new Pose2d(-80, -60, Math.toRadians(180));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD_ONE);
                builder.lineToLinearHeading(RIGHT_PRELOAD_TWO);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
                break;
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD);
                break;
        }
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void boardScore() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_BOARD);
                break;
            case "LEFT":
                builder.lineToLinearHeading(LEFT_BOARD);
                break;
        }
        builder.addTemporalMarker(.2, robot.getArm()::armScore);
        builder.addTemporalMarker(.2, robot.getSlides()::slideFirstLayer);
        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
        this.robot.getDrive().followTrajectorySequence(builder.build());
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
        this.robot.getCamera().initTargetingCamera();
        this.initialPosition = new Pose2d(-34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

//        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
//                .lineToLinearHeading(new Pose2d(80, -57, Math.toRadians(360)))
//                .build();

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPositionBlue());
            randomization = String.valueOf(this.robot.getCamera().getStartingPositionBlue());
            this.telemetry.update();
        }

        scorePreloadOne();
        boardScore();
        sleep(250);
        this.robot.getClaw().open();
        sleep(250);
        park();
    }

}
