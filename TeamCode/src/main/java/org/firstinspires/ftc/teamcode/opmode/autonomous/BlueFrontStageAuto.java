package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK1;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK2;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK3;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK4;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK6;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Config
@Autonomous(name = "Blue FrontStage Auto (2+3)", group = "Competition", preselectTeleOp = "Main TeleOp")
public class BlueFrontStageAuto extends AutoBase {
    public static final Pose2d DROP_1_PART_2 = new Pose2d(-36, 33.5, Math.toRadians(0));

    public static final Pose2d DROP_1 = new Pose2d(-32,33.5,Math.toRadians(0));
    public static final Pose2d DROP_2 = new Pose2d(-39.7, 33.5, Math.toRadians(-90));
    public static final Pose2d DROP_2_PART_2 = new Pose2d(-39.7,36.5, Math.toRadians(-90));
    public static final Pose2d DROP_3 = new Pose2d(-46.7, 50.5, Math.toRadians(-90));
    public static final Pose2d DROP_1M = new Pose2d(-36, 45, Math.toRadians(-90));

    public static final Pose2d DROP_2M = new Pose2d(-48.5, 30, Math.toRadians(-90));

    public static final Pose2d DROP_3M = new Pose2d(-48.7, 35.9, Math.toRadians(-90));

    public static final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(53.3, 38.3, Math.toRadians(8));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(53, 34, Math.toRadians(8));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(53.6, 23.5, Math.toRadians(6));//817

    public static final Pose2d STACK_LOCATION = new Pose2d(-52, 29.6, Math.toRadians(180));

    public static final Pose2d POST_SCORING_SPLINE_END = new Pose2d(24, 12.4, Math.toRadians(10));//-36

    public static final Pose2d POST_DROP_POS = new Pose2d(-45, 59.5, Math.toRadians(180));

    public static final Pose2d POST_DROP_POS_PART2 = new Pose2d(-33, 59.5, Math.toRadians(180));

    public static final Pose2d PRE_DEPOSIT_POS = new Pose2d(33, 59.5, Math.toRadians(180));

    public static final Pose2d PARK_1 = new Pose2d(45,58,Math.toRadians(-180));

    public static final Pose2d PARK_2 = new Pose2d(58,58,Math.toRadians(-180));

    @Override
    public void createTrajectories() {
        // set start position
        Pose2d start = new Pose2d(-36, 61.5, Math.toRadians(-90));
        robot.drive.setPoseEstimate(start);
        robot.camera.setAlliance(CenterStageCommon.Alliance.Blue);
    }

    @Override
    public void followTrajectories() {
        TrajectorySequenceBuilder builder = null;
        switch (macroState) {
            case 0:
                builder = this.robot.getTrajectorySequenceBuilder();
                switch (teamPropLocation) {
                    case 1:
                        builder.lineToLinearHeading(DROP_1M);
                        builder.lineToLinearHeading(DROP_1);
                        break;
                    case 2:
                        builder.lineToLinearHeading(DROP_2);
                        break;
                    case 3:
                        builder.lineToLinearHeading(DROP_3M);
//                        builder.lineToLinearHeading(DROP_3);
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
                if (getRuntime() < macroTime + 0.18) {
                    robot.intake.setDcMotor(-0.23);
                }
                else{
                    builder = this.robot.getTrajectorySequenceBuilder();
                    robot.intake.setDcMotor(0);
                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToLinearHeading(DROP_1_PART_2);
                            break;
                        case 2:
                            builder.lineToLinearHeading(DROP_2_PART_2);
                            break;
                        case 3:
                            builder.lineToLinearHeading(DROP_3);
                            break;
                    }
                    robot.arm.setDoor(OPEN);
                    builder.lineToLinearHeading(STACK_LOCATION.plus(new Pose2d(-5.4,2.5))).waitSeconds(.01);


                    robot.intake.setDcMotor(0.74);
                    robot.intake.setpos(STACK6);
                    macroTime = getRuntime();
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                // if intake is done move on
                break;
            case 3:
                // if drive is done move on
                if (!robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    macroState++;
                }
                break;


            case 4:
                if (getRuntime() > macroTime + 0.06) {
                    robot.arm.setDoor(CLOSE);
                    robot.intake.setDcMotor(-0.5);
                    builder = this.robot.getTrajectorySequenceBuilder();

                    switch (teamPropLocation) {
                        case 1:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(),Math.toRadians(0));
                            break;
                        case 2:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(),Math.toRadians(0));
                            break;
                        case 3:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(),Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 5:
                // if drive is done move on
                if (getRuntime() > macroTime + 4.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1-70, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 6:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;

            //Stack run 2
            case 7:
                robot.resetMacro(0, getRuntime());
                if (getRuntime() > macroTime + 3.4 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.splineToConstantHeading(PRE_DEPOSIT_POS.plus(new Pose2d(0,-2)).vec(), Math.toRadians(180));
                    builder.lineToConstantHeading(POST_DROP_POS.plus(new Pose2d(0,-2)).vec());
                    builder.splineToConstantHeading(STACK_LOCATION.plus(new Pose2d(-3.9, -3.7)).vec(), Math.toRadians(180));
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            //waits for the robot to fin  the trajectory
            case 8:
                robot.resetMacro(0, getRuntime());
                robot.arm.setDoor(OPEN);
                robot.intake.setDcMotor(0.54);
                robot.intake.setpos(STACK4);
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //3rd and 4th pixels off the stack are in-taken by this
            case 9:
                robot.intake.setDcMotor(0.54);
                robot.arm.setDoor(OPEN);
                robot.intake.setpos(STACK3);
                macroTime = getRuntime();
                macroState++;
                break;

            case 10:
                if (getRuntime() > macroTime + 0.6) {
                    robot.arm.setDoor(CLOSE);
                    robot.intake.setDcMotor(-0.45);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    //builder.splineToLinearHeading(POST_DROP_POS, Math.toRadians(0));
                    //builder.lineToLinearHeading(POST_DROP_POS_PART2.plus(new Pose2d(0,2)));


                    switch (teamPropLocation) {
                        case 1:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(),Math.toRadians(0));
                            break;
                        case 2:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(),Math.toRadians(0));
                            break;
                        case 3:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(),Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 11:
                // if drive is done move on
                if (getRuntime() > macroTime + 6.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 20, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 12:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;

            //stack run 3
            case 13:
                robot.resetMacro(0, getRuntime());
                if (getRuntime() > macroTime + 2.4 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.splineToConstantHeading(PRE_DEPOSIT_POS.vec(), Math.toRadians(180));
                    builder.lineToLinearHeading(POST_DROP_POS);
                    builder.splineToConstantHeading(STACK_LOCATION.plus(new Pose2d(-1.5)).vec(), Math.toRadians(180));
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;

                //waits for the robot to fin  the trajectory
            case 14:
                robot.resetMacro(0, getRuntime());
                robot.arm.setDoor(OPEN);
                robot.intake.setDcMotor(0.54);
                robot.intake.setpos(STACK2);
                if (!robot.drive.isBusy()) {
                    macroState++;
                }
                break;
            //3rd and 4th pixels off the stack are intaken by this
            case 15:
                robot.intake.setDcMotor(0.54);
                robot.arm.setDoor(OPEN);
                robot.intake.setpos(STACK1);
                macroTime = getRuntime();
                macroState++;
                break;
            case 16:
                if (getRuntime() > macroTime + 0.6) {
                    robot.arm.setDoor(CLOSE);
                    robot.intake.setDcMotor(-0.45);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    //builder.splineToLinearHeading(POST_DROP_POS, Math.toRadians(0));
                    //builder.lineToLinearHeading(POST_DROP_POS_PART2.plus(new Pose2d(0,2)));


                    switch (teamPropLocation) {
                        case 1:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(),Math.toRadians(0));
                            break;
                        case 2:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(),Math.toRadians(0));
                            break;
                        case 3:
                            builder.setTangent(Math.toRadians(90));
                            builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(),Math.toRadians(0));
                            builder.lineToConstantHeading(PRE_DEPOSIT_POS.vec());
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(),Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 17:
                // if drive is done move on
                if (getRuntime() > macroTime + 6.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 20, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 18:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;

            case 19:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState == 0) {
                    macroState = -1;
                }
        }
    }
}
