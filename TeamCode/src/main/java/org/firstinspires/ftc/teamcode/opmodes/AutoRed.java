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

@Autonomous(name = "autoRed")
public class AutoRed extends LinearOpMode {
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
    final static Pose2d LEFT_BOARD = new Pose2d(78, -22, Math.toRadians(360));
    final static Pose2d CENTER_BOARD = new Pose2d(78, -33, Math.toRadians(360));
    final static Pose2d RIGHT_BOARD = new Pose2d(76, -39, Math.toRadians(360));
    //Parka
    final static Pose2d PARK = new Pose2d(60, -58, Math.toRadians(360));
    final static Pose2d PARK2 = new Pose2d(80, -60, Math.toRadians(360));
    final static Pose2d PARKLEFT = new Pose2d(50, -15, Math.toRadians(360));
    final static Pose2d PARKLEFT2 = new Pose2d(75, -2, Math.toRadians(360));

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

    protected void park() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch(parkLocation) {
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
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
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
        this.robot.getClaw().open();
        sleep(250);
        park();
    }

}
