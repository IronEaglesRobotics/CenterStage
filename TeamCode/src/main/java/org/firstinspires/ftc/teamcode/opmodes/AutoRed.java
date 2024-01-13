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
@Autonomous(name = "autoRed")
public class AutoRed extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization;

    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_ONE = new Pose2d(40, -37.5, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(29.5, -29, Math.toRadians(360));
    final static Pose2d CENTER_PRELOAD = new Pose2d(33, -28, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(43, -35, Math.toRadians(270));
    //Board Scores
    final static Pose2d LEFT_BOARD = new Pose2d(74.3, -26.5, Math.toRadians(355));
    final static Pose2d CENTER_BOARD = new Pose2d(74.7, -36.3, Math.toRadians(355));
    final static Pose2d RIGHT_BOARD = new Pose2d(74.3, -40, Math.toRadians(355));

    //Park
    final static Pose2d BACK_OFF =  new Pose2d(60,-58,Math.toRadians(360));
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
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void boardScore() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_BOARD);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_BOARD);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD);
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
        builder.addTemporalMarker(.1, robot.getWrist()::wristPickup);
        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.robot.getCamera().initTargetingCamera();
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

//        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
//                .lineToLinearHeading(new Pose2d(80, -57, Math.toRadians(360)))
//                .build();

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
        park();
    }

}
