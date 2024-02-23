package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK1;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK2;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK3;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK4;
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

    public static final Pose2d DROP_1 = new Pose2d(16, -32.5, Math.toRadians(180));

    public static final Pose2d DROP_1M = new Pose2d(11.5, -32.5, Math.toRadians(180));

    public static final Pose2d DROP_2 = new Pose2d(15.6, -33.5, Math.toRadians(90));

    public static final Pose2d ALINE = new Pose2d(51,-32.5, Math.toRadians(180));

    public static final Pose2d DROP_3 = new Pose2d(25, -42.5, Math.toRadians(90));
    public static final Pose2d DEPOSIT_PRELOAD_1 = new Pose2d(52, -26.3, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_2 = new Pose2d(52.3, -32.5, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_3 = new Pose2d(51, -40.7, Math.toRadians(180));

    public static  final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(53, -35.3, Math.toRadians(180));//187

    public static  final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(51.6, -29.5, Math.toRadians(180));//187

    public static  final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(51.4, -34.5, Math.toRadians(180));//187


    //public static final Vector2d POST_SCORING_SPLINE_END = new Vector2d(24, -8.5);//-36
    public static final Pose2d POST_SCORING_SPLINE_END = new Pose2d(27, -11, Math.toRadians(180));//-36

    public static final Pose2d STACK_LOCATION_1 = new Pose2d(-56, -11, Math.toRadians(180));

    public static final Pose2d STACK_LOCATION_2 = new Pose2d(-56.2, -11, Math.toRadians(180));

    public static final Pose2d STACK_LOCATION_3 = new Pose2d(-56.2, -11, Math.toRadians(185));

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
                        builder.lineToLinearHeading(DROP_1M);
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
                if (getRuntime() < macroTime + 0.26) {
                    robot.intake.setDcMotor(-0.18);
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
                            //robot.extendMacro(Slides.mini_tier1 -40, getRuntime());
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
                    robot.extendMacro(Slides.mini_tier1 - 20, getRuntime());
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
                if (getRuntime() > macroTime + 1.5 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    robot.intake.setDcMotor(0);
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.splineToConstantHeading(POST_SCORING_SPLINE_END.vec(),Math.toRadians(180));
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToConstantHeading(STACK_LOCATION_1.vec().plus(new Vector2d(-1.85)));
                            break;
                        case 2:
                            builder.lineToConstantHeading(STACK_LOCATION_2.vec().plus(new Vector2d(-1)));
                            break;
                        case 3:
                            builder.lineToConstantHeading(STACK_LOCATION_3.vec().plus(new Vector2d(-3)));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 5:
                robot.resetMacro(0, getRuntime());
                robot.intake.setDcMotor(0.58);
                robot.intake.setpos(STACK5);
                if (!robot.drive.isBusy()) {

                    macroState++;
                }
                break;
            //First 2 pixels off the stack are intaken by this
            case 6:
                robot.intake.setDcMotor(0.58);
                robot.intake.setpos(STACK4);
                macroTime = getRuntime();
                macroState++;
                break;
            //goes back to the easel
            case 7:
                if (getRuntime() > macroTime + 1.2) {
                    robot.arm.setDoor(Arm.DoorPosition.CLOSE);
                    robot.intake.setDcMotor(-0.35);
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
                    robot.extendMacro(Slides.mini_tier1 , getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 9:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 , getRuntime());
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
                if (getRuntime() > macroTime + 1.5 || robot.macroState >= 4) {
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.splineToConstantHeading(POST_SCORING_SPLINE_END.vec(),Math.toRadians(180));
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToConstantHeading(STACK_LOCATION_1.vec().plus(new Vector2d(-2)));
                            break;
                        case 2:
                            builder.lineToConstantHeading(STACK_LOCATION_2.vec().plus(new Vector2d(-3)));
                            break;
                        case 3:
                            builder.lineToConstantHeading(STACK_LOCATION_3.vec().plus(new Vector2d(-2.8)));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 11:
                robot.resetMacro(0, getRuntime());
                robot.intake.setpos(STACK3);
                robot.intake.setDcMotor(0.58);
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //3rd and 4th pixels off the stack are intaken by this
            case 12:
                robot.intake.setDcMotor(0.58);
                robot.intake.setpos(STACK2);
                macroTime = getRuntime();
                macroState++;
                break;
            //goes back to the easel
            case 13:
                if (getRuntime() > macroTime + 1) {
                    robot.arm.setDoor(Arm.DoorPosition.CLOSE);
                    robot.intake.setDcMotor(-0.35);
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
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
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