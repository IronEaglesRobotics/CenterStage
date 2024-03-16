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

@Autonomous(name = "AutoRedFar1+0")
public class AutoRedFaronepluszero extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization;
    private String parkLocation;
    private int delay = 10000;

    //Pose2ds
    //Preloads
    final static Pose2d RIGHT_PRELOAD_ONE = new Pose2d(-40, -33.5, Math.toRadians(230));
    final static Pose2d RIGHT_PRELOAD_TWO = new Pose2d(-29.5, -31, Math.toRadians(200));
    final static Pose2d RIGHT_PRELOAD_GETOUT = new Pose2d(-43, -42, Math.toRadians(230));
    final static Pose2d CENTER_PRELOAD = new Pose2d(-35, -28, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD = new Pose2d(-47, -35, Math.toRadians(270));
    //Ready Truss
    final static Pose2d LEAVE = new Pose2d(-43, -40, Math.toRadians(270));
    final static Pose2d READY_TRUSS = new Pose2d(-43, -57, Math.toRadians(0));
    final static Pose2d TO_BOARD = new Pose2d(30, -57, Math.toRadians(0));
    final static Pose2d READY_SCORE = new Pose2d(37, -34, Math.toRadians(0));
    final static Pose2d SCORE_BOARD_LEFT = new Pose2d(54, -27, Math.toRadians(5));
    final static Pose2d SCORE_BOARD_MID = new Pose2d(54, -32, Math.toRadians(0));
    final static Pose2d SCORE_BOARD_RIGHT = new Pose2d(54, -41, Math.toRadians(0));
    final static Pose2d PARK = new Pose2d(45, -60, Math.toRadians(0));
    final static Pose2d PARK2 = new Pose2d(60, -63, Math.toRadians(0));
    final static Pose2d PARKLEFT = new Pose2d(45, -12, Math.toRadians(0));
    final static Pose2d PARKLEFT2 = new Pose2d(60, -12, Math.toRadians(0));

    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD_ONE);
                builder.lineToLinearHeading(RIGHT_PRELOAD_TWO);
                builder.lineToLinearHeading(RIGHT_PRELOAD_GETOUT);
                break;
        }
        builder.addTemporalMarker(.4,robot.getArm()::armScore);
//        builder.addTemporalMarker(2,robot.getArm()::armRest);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void goBackToWhereYouCameFrom() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(LEAVE);
//        builder.lineToLinearHeading(READY_TRUSS);
//        builder.lineToLinearHeading(TO_BOARD);
        builder.addTemporalMarker(.3,robot.getArm()::armRest);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void scoreBoard(){
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(READY_SCORE);

        builder.addTemporalMarker(.2, robot.getArm()::armSecondaryScore);
        builder.addTemporalMarker(.2, robot.getSlides()::slideAutoStacks);
        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void score(){
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(SCORE_BOARD_LEFT,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                break;
            case "CENTER":
                builder.lineToLinearHeading(SCORE_BOARD_MID,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                break;
            case "RIGHT":
                builder.lineToLinearHeading(SCORE_BOARD_RIGHT,
                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(20));
                break;
        }
        builder.addTemporalMarker(1.8, robot.getClaw()::open);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void park(){
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
        this.robot.getDrive().followTrajectorySequence(builder.build());

    }

    protected void parkLocation(){
    if (gamepad2.dpad_left) {
        parkLocation="LEFT";
    } else if (gamepad2.dpad_right) {
        parkLocation = "RIGHT";
    }}

    protected void delaySet(){
        if (gamepad2.dpad_up && delay<13000) {
            delay+=1000;
            sleep(100);
        } else if (gamepad2.dpad_down && delay>0) {
            delay-=1000;
            sleep(100);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
//        this.robot.getCamera().initTargetingCamera();
        this.initialPosition = new Pose2d(-34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            parkLocation();
            delaySet();
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPosition());
            randomization = String.valueOf(this.robot.getCamera().getStartingPosition());
            this.telemetry.addData("Park Position", parkLocation);
            this.telemetry.addData("Delay", delay);
            this.telemetry.update();

        }
//        sleep(delay);
        scorePreloadOne();
        goBackToWhereYouCameFrom();
//        scoreBoard();
//        score();
//        sleep(500);
//        park();
    }

}
