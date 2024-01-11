package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK3;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name = "Red Backstage Auto", group = "Competition", preselectTeleOp = "Main TeleOp")
public class RedBackStageAuto extends AutoBase {
    public static final Pose2d DROP_1 = new Pose2d(12, -33.3, Math.toRadians(180));
    public static final Pose2d DROP_2 = new Pose2d(12, -33.2, Math.toRadians(90));

    public static final Pose2d ALINE = new Pose2d(12,-37.5, Math.toRadians(90));

    public static final Pose2d DROP_3 = new Pose2d(12, -33.2, Math.toRadians(0));
    public static final Pose2d DEPOSIT_PRELOAD_1 = new Pose2d(52.2, -27.5, Math.toRadians(181));
    public static final Pose2d DEPOSIT_PRELOAD_2 = new Pose2d(52, -32.5, Math.toRadians(181));
    public static final Pose2d DEPOSIT_PRELOAD_3 = new Pose2d(52.2, -35.5, Math.toRadians(181));

    public static  final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(51.2, -35, Math.toRadians(187));

    public static  final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(51.2, -29, Math.toRadians(187));

    public static  final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(51.2, -29, Math.toRadians(187));


    public static final Vector2d POST_SCORING_SPLINE_END = new Vector2d(12, -8.5);//-36

    public static final Pose2d STACK_LOCATION = new Pose2d(-56.6, -12, Math.toRadians(180));

    @Override
    public void createTrajectories() {
        // set start position
        Pose2d start = new Pose2d(12, -61.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(start);
    }

    @Override
    public void followTrajectories() {
        TrajectorySequenceBuilder builder;
        switch (macroState) {
            case 0:
                builder = this.robot.getTrajectorySequenceBuilder();
                switch (teamPropLocation) {
                    case 1:
                        builder.lineToLinearHeading(DROP_1);
                        break;
                    case 2:
                        builder.lineToLinearHeading(DROP_2);
                        break;
                    case 3:
                        builder.lineToLinearHeading(DROP_3);
                        break;
                }
                this.robot.drive.followTrajectorySequenceAsync(builder.build());
                macroState++;
                break;
            // DRIVE TO TAPE
            case 1:
                // if drive is done move on
                if (!robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    macroState++;
                }
                break;
            // RUN INTAKE
            case 2:
                // intake
                if (getRuntime() < macroTime + 0.3) {
                    robot.intake.setDcMotor(-0.21);
                }
                // if intake is done move on
                else {
                    robot.intake.setDcMotor(0);
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                    builder = this.robot.getTrajectorySequenceBuilder();
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToLinearHeading(DEPOSIT_PRELOAD_1);
                            break;
                        case 2:
                            builder.lineToLinearHeading(DEPOSIT_PRELOAD_2);
                            break;
                        case 3:
                            builder.lineToLinearHeading(DEPOSIT_PRELOAD_3);
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            // EXTEND AND MOVE TO BACKBOARD
            case 3:
                // extend macro
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                }
                // if macro and drive are done, move on
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;
            case 4:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.lineToConstantHeading(STACK_LOCATION.vec());
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 5:
                robot.resetMacro(0, getRuntime());
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //First 2 pixels off the stack are intaken by this
            case 6:
                robot.intake.setDcMotor(0.49);
                robot.intake.setpos(STACK5);
                macroTime = getRuntime();
                macroState++;
                break;
            //gose back to the esile
            case 7:
                if (getRuntime() > macroTime + 1.2) {
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec());
                            break;
                        case 2:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec());
                            break;
                        case 3:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec());
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            case 8:
                // if drive is done move on
                if (!robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                    macroState++;
                }
                break;
            case 9:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;
            case 10:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.lineToConstantHeading(STACK_LOCATION.vec());
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            case 11:
                robot.resetMacro(0, getRuntime());
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //Geting the 2nd and 3rd pixel
            case 12:
                robot.intake.setDcMotor(0.49);
                robot.intake.setpos(STACK3);
                macroTime = getRuntime();
                macroState++;
                break;
            case 13:
                if (getRuntime() > macroTime + 1.2) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec());
                            break;
                        case 2:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec());
                            break;
                        case 3:
                            builder.lineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec());
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            case 14:
                macroState = -1;
                // PARK ROBOT
//            case 6:
//                // reset macro'
//                if (robot.macroState != 0) {
//                    robot.resetMacro(0, getRuntime());
//                }
//                // if macro and drive are done, end auto
//                if (robot.macroState == 0 && !robot.drive.isBusy()) {
//                    macroState=-1;
//                }
//                break;
        }
    }
}