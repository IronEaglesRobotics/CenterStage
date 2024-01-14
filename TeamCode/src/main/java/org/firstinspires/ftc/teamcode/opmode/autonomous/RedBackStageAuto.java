package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK2;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK3;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Config
@Autonomous(name = "Red Backstage Auto(2+4)", group = "Competition", preselectTeleOp = "Main TeleOp")
public class RedBackStageAuto extends AutoBase {
    public static final Pose2d DROP_1 = new Pose2d(12, -32.5, Math.toRadians(180));
    public static final Pose2d DROP_2 = new Pose2d(13.7, -32.5, Math.toRadians(90));

    public static final Pose2d ALINE = new Pose2d(51,-32.5, Math.toRadians(180));

    public static final Pose2d DROP_3 = new Pose2d(25, -45.5, Math.toRadians(90));
    public static final Pose2d DEPOSIT_PRELOAD_1 = new Pose2d(52, -27.5, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_2 = new Pose2d(53.5, -32.5, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_3 = new Pose2d(51.3, -39.5, Math.toRadians(180));

    public static  final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(50.3, -35.3, Math.toRadians(180));//187

    public static  final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(52, -29, Math.toRadians(180));//187

    public static  final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(50.6, -32, Math.toRadians(180));//817


    //public static final Vector2d POST_SCORING_SPLINE_END = new Vector2d(24, -8.5);//-36
    public static final Pose2d POST_SCORING_SPLINE_END = new Pose2d(24, -12.5, Math.toRadians(180));//-36

    public static final Pose2d STACK_LOCATION = new Pose2d(-56.8, -12.5, Math.toRadians(180));

    @Override
    public void createTrajectories() {
        // set start position
        Pose2d start = new Pose2d(12, -61.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(start);
        robot.camera.setAlliance(CenterStageCommon.Alliance.Red);
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
                if (getRuntime() < macroTime + 0.28) {
                    robot.intake.setDcMotor(-0.22);
                }
                // if intake is done move on
                else {
                    robot.intake.setDcMotor(0);
                    robot.arm.setDoor(Arm.DoorPosition.CLOSE);
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //Scores yellow pixel
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
                    macroTime = getRuntime();
                }
                break;
            // STACK RUN 1 -------------------------
            case 4:
                robot.resetMacro(0, getRuntime());
                if (getRuntime() > macroTime + 1.4 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.splineToConstantHeading(POST_SCORING_SPLINE_END.vec(),Math.toRadians(180));
                    builder.lineToConstantHeading(STACK_LOCATION.vec());
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 5:
                robot.resetMacro(0, getRuntime());
                robot.intake.setDcMotor(0.44);
                robot.intake.setpos(STACK5);
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //First 2 pixels off the stack are intaken by this
            case 6:
                robot.intake.setDcMotor(0.44);
                robot.intake.setpos(STACK5);
                macroTime = getRuntime();
                macroState++;
                break;
            //goes back to the easel
            case 7:
                if (getRuntime() > macroTime + 0.5) {
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END.vec());
                    switch (teamPropLocation) {
                        case 1:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(),Math.toRadians(0));
                            break;
                        case 2:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(),Math.toRadians(0));
                            break;
                        case 3:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(),Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 8:
                // if drive is done move on
                if (getRuntime() > macroTime + 1.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 9:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            // STACK RUN 2
            case 10:
                robot.resetMacro(0, getRuntime());
                if (getRuntime() > macroTime + 1.4 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.splineToConstantHeading(POST_SCORING_SPLINE_END.vec(),Math.toRadians(180));
                    builder.lineToConstantHeading(STACK_LOCATION.vec());
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 11:
                robot.resetMacro(0, getRuntime());
                robot.intake.setDcMotor(0.44);
                robot.intake.setpos(STACK3);
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //First 2 pixels off the stack are intaken by this
            case 12:
                robot.intake.setDcMotor(0.44);
                robot.intake.setpos(STACK3);
                macroTime = getRuntime();
                macroState++;
                break;
            //goes back to the easel
            case 13:
                if (getRuntime() > macroTime + 0.5) {
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END.vec());
                    switch (teamPropLocation) {
                        case 1:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(),Math.toRadians(0));
                            break;
                        case 2:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(),Math.toRadians(0));
                            break;
                        case 3:
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(),Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 14:
                // if drive is done move on
                if (getRuntime() > macroTime + 1.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 180, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 15:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;

            // PARK ROBOT
            case 16:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState == 0) {
                    macroState = -1;
                }

                // PARK ROBOT
//
        }
    }
}