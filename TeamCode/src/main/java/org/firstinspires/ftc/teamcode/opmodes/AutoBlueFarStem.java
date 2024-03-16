package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "autoBlueFar")
public class AutoBlueFarStem extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;
    private String randomization;
    private String parkLocation;
    private int delay = 10000;

    //Pose2ds
    //Preloads
    final static Pose2d LEFT_PRELOAD_ONE = new Pose2d(40, -37.5, Math.toRadians(270));
    final static Pose2d LEFT_PRELOAD_TWO = new Pose2d(29.5, -29, Math.toRadians(360));
    final static Pose2d LEFT_PRELOAD_GETOUT = new Pose2d(43, -42, Math.toRadians(330));
    final static Pose2d CENTER_PRELOAD = new Pose2d(35, -27, Math.toRadians(270));
    final static Pose2d RIGHT_PRELOAD = new Pose2d(44, -35, Math.toRadians(270));
    //Ready Truss
    final static Pose2d READY_TRUSS = new Pose2d(43, -57, Math.toRadians(180));
    final static Pose2d READY_TRUSSTEMP = new Pose2d(35, -57, Math.toRadians(180));
    final static Pose2d TO_BOARD = new Pose2d(-35, -57, Math.toRadians(180));
    final static Pose2d SCORE_BOARD_LEFT = new Pose2d(-52, -38, Math.toRadians(180));
    final static Pose2d SCORE_BOARD_MID = new Pose2d(-52, -32, Math.toRadians(180));
    final static Pose2d SCORE_BOARD_RIGHT = new Pose2d(-52, -27, Math.toRadians(180));
    final static Pose2d PARK = new Pose2d(-55, -56, Math.toRadians(180));
    final static Pose2d PARK2 = new Pose2d(-60, -59, Math.toRadians(180));
    final static Pose2d PARKLEFT = new Pose2d(-45, -10, Math.toRadians(180));
    final static Pose2d PARKLEFT2 = new Pose2d(-60, -10, Math.toRadians(180));


    protected void scorePreloadOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        switch (randomization) {
            case "LEFT":
                builder.lineToLinearHeading(LEFT_PRELOAD_ONE);
                builder.lineToLinearHeading(LEFT_PRELOAD_TWO);
                builder.lineToLinearHeading(LEFT_PRELOAD_GETOUT);
                break;
            case "CENTER":
                builder.lineToLinearHeading(CENTER_PRELOAD);
                break;
            case "RIGHT":
                builder.lineToLinearHeading(RIGHT_PRELOAD);
                break;
        }
        builder.addTemporalMarker(.3, this.robot.getArm()::armScore);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    protected void goBackToWhereYouCameFrom() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.lineToLinearHeading(READY_TRUSSTEMP);
//        builder.lineToLinearHeading(TO_BOARD,
//                MecanumDrive.getVelocityConstraint(70, 70, DriveConstants.TRACK_WIDTH),
//                MecanumDrive.getAccelerationConstraint(70));
        builder.addTemporalMarker(.3,robot.getArm()::armRest);
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

//    protected void scoreBoard(){
//        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
//        switch (randomization) {
//            case "LEFT":
//                builder.lineToLinearHeading(SCORE_BOARD_LEFT,
//                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
//                        MecanumDrive.getAccelerationConstraint(20));
//                break;
//            case "CENTER":
//                builder.lineToLinearHeading(SCORE_BOARD_MID,
//                        MecanumDrive.getVelocityConstraint(20, 20, DriveConstants.TRACK_WIDTH),
//                        MecanumDrive.getAccelerationConstraint(20));
//                break;
//            case "RIGHT":
//                builder.lineToLinearHeading(SCORE_BOARD_RIGHT,
//                        MecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
//                        MecanumDrive.getAccelerationConstraint(30));
//                break;
//        }
//        builder.addTemporalMarker(.2, robot.getArm()::armScore);
//        builder.addTemporalMarker(.2, robot.getSlides()::slideAutoStacks);
//        builder.addTemporalMarker(.2, robot.getWrist()::wristScore);
//        this.robot.getDrive().followTrajectorySequence(builder.build());
//    }
//
//    protected void park(){
//        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
//        switch(parkLocation) {
//            case "RIGHT":
//                builder.lineToLinearHeading(PARKLEFT);
//                builder.lineToLinearHeading(PARKLEFT2);
//                break;
//            case "LEFT":
//                builder.lineToLinearHeading(PARK);
//                builder.lineToLinearHeading(PARK2);
//                break;
//        }
//        builder.addTemporalMarker(.1, robot.getArm()::armRest);
//        builder.addTemporalMarker(.1, robot.getClaw()::close);
//        builder.addTemporalMarker(.1, robot.getWrist()::wristPickup);
//        builder.addTemporalMarker(.1, robot.getSlides()::slideDown);
//        this.robot.getDrive().followTrajectorySequence(builder.build());
//
//    }

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
            this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPositionBlue());
            randomization = String.valueOf(this.robot.getCamera().getStartingPositionBlue());
            if (gamepad2.dpad_left) {
                parkLocation="LEFT";
            } else if (gamepad2.dpad_right) {
                parkLocation = "RIGHT";
            }
            delaySet();
            this.telemetry.addData("Park Position", parkLocation);
            this.telemetry.addData("Delay", delay);
            this.telemetry.update();
        }
        sleep(delay);
        scorePreloadOne();
        goBackToWhereYouCameFrom();
//        scoreBoard();
//        sleep(500);
//        this.robot.getClaw().open();
//        sleep(500);
//        park();
    }

}
