package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.Configurables.AuToDeV.TEMP1;
import static org.firstinspires.ftc.teamcode.util.Configurables.AuToDeV.TEMP2;
import static org.firstinspires.ftc.teamcode.util.Configurables.AuToDeV.TEMP3;
import static org.firstinspires.ftc.teamcode.util.Configurables.AuToDeV.TEMP4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "autoRed")
public class AutoRed extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization = "CENTER";
    private String parkLocation = "LEFT";

    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_ONE = new Pose2d(40, -37.5, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(29.5, -32, Math.toRadians(360));
    final static Pose2d CENTER_PRELOAD = new Pose2d(24, -28, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(36, -30, Math.toRadians(270));
    //Board Scores
    final static Pose2d LEFT_BOARD = new Pose2d(50, -26.5, Math.toRadians(358));
    final static Pose2d CENTER_BOARD = new Pose2d(50, -36.3, Math.toRadians(358));
    final static Pose2d RIGHT_BOARD = new Pose2d(50, -40, Math.toRadians(358));

    //Park
    final static Pose2d PARK = new Pose2d(60, -58, Math.toRadians(360));
    final static Pose2d PARK2 = new Pose2d(80, -60, Math.toRadians(360));
    final static Pose2d PARKLEFT = new Pose2d(60, -6, Math.toRadians(360));
    final static Pose2d PARKLEFT2 = new Pose2d(80, -6, Math.toRadians(360));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD_ONE);
                builder.lineToLinearHeading(LEFT_PRELOAD_TWO);
                break;
            case "CENTER":
                builder.lineToLinearHeading(TEMP1);
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
                builder.lineToLinearHeading(TEMP2);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_BOARD);
                break;
        }
        builder.addTemporalMarker(.2, robot.getArm()::armScore);
        builder.addTemporalMarker(.2, robot.getSlides()::slideFirstLayer);
        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
        this.robot.getDrive().followTrajectorySequenceAsync(builder.build());
        while (this.robot.getDrive().isBusy()) {
            robot.update();
        }
    }

    protected void park() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch(parkLocation) {
            case "LEFT":
                builder.lineToLinearHeading(TEMP3);
                builder.lineToLinearHeading(TEMP4);
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
        this.initialPosition = new Pose2d(24, -60, Math.toRadians(270));
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
        scorePreloadOne();
        boardScore();
        sleep(250);
        this.robot.getClaw().open();
        sleep(250);
//        park();
    }

}
