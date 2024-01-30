package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK1;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK2;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK3;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK4;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Config
@Autonomous(name = "Red FrontStage Auto(2+5)", group = "Competition", preselectTeleOp = "Main TeleOp")
public class RedFrontStageAuto extends AutoBase {
    public static final Pose2d DROP_1 = new Pose2d(-50, -45.5, Math.toRadians(90));
    public static final Pose2d DROP_2 = new Pose2d(-39.7, -33.5, Math.toRadians(90));
    public static final Pose2d DROP_3 = new Pose2d(-33.5, -40.5, Math.toRadians(0));

    public static final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(50.3, -35.3, Math.toRadians(188));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(50.5, -33, Math.toRadians(188));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(50.6, -32, Math.toRadians(188));//817

    public static final Pose2d STACK_LOCATION = new Pose2d(-52, -34.4, Math.toRadians(180));

    public static final Pose2d POST_SCORING_SPLINE_END = new Pose2d(24, -12.4, Math.toRadians(190));//-36

    public static final Pose2d POST_DROP_POS = new Pose2d(-45, -57.5, Math.toRadians(180));

    public static final Pose2d POST_DROP_POS_PART2 = new Pose2d(-33, -60.5, Math.toRadians(180));

    public static final Pose2d PRE_DEPOSIT_POS = new Pose2d(33, -60.5, Math.toRadians(180));

    @Override
    public void createTrajectories() {
        // set start position
        Pose2d start = new Pose2d(-36, -61.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(start);
        robot.camera.setAlliance(CenterStageCommon.Alliance.Red);
    }

    @Override
    public void followTrajectories() {
        TrajectorySequenceBuilder builder = null;
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
                if (getRuntime() < macroTime + 0.22) {
                    robot.intake.setDcMotor(-0.15);
                }
                else{
                    builder = this.robot.getTrajectorySequenceBuilder();
                    robot.intake.setDcMotor(0);
                    builder.lineToLinearHeading(STACK_LOCATION.plus(new Pose2d(0,-1.5)));
                    robot.arm.setDoor(OPEN);
                    robot.intake.setDcMotor(0.54);
                    robot.intake.setpos(STACK5);
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
                if (getRuntime() > macroTime + 0.2) {
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.lineToLinearHeading(POST_DROP_POS);
                    builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(), Math.toRadians(0));

                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(), Math.toRadians(0));
                            break;
                        case 2:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(), Math.toRadians(0));
                            break;
                        case 3:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(), Math.toRadians(0));
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
                    robot.extendMacro(Slides.mini_tier1 + 180, getRuntime());
                    macroState++;
                }
                break;
            //extending the macro and about to score
            case 6:
                if (robot.macroState != 0) {
                    robot.extendMacro(Slides.mini_tier1 + 80, getRuntime());
                }
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    robot.resetMacro(0, getRuntime());
                    macroState++;
                }
                break;

            //Stack run 2
            case 7:
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
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.splineToSplineHeading(POST_DROP_POS, Math.toRadians(0));
                    builder.splineToConstantHeading(POST_DROP_POS_PART2.vec(), Math.toRadians(0));


                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(), Math.toRadians(0));
                            break;
                        case 2:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(), Math.toRadians(0));
                            break;
                        case 3:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(), Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 11:
                // if drive is done move on
                if (getRuntime() > macroTime + 4.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 180, getRuntime());
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
                    robot.intake.setDcMotor(0);
                    builder = this.robot.getTrajectorySequenceBuilder();
                    //builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    builder.splineToSplineHeading(POST_DROP_POS, Math.toRadians(0));

                    switch (teamPropLocation) {
                        case 1:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_1.vec(), Math.toRadians(0));
                            break;
                        case 2:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_2.vec(), Math.toRadians(0));
                            break;
                        case 3:
                            builder.lineToLinearHeading(PRE_DEPOSIT_POS);
                            builder.splineToConstantHeading(DEPOSIT_WHITE_STACKS_3.vec(), Math.toRadians(0));
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                    macroTime = getRuntime();
                }
                break;
            case 17:
                // if drive is done move on
                if (getRuntime() > macroTime + 4.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1 + 180, getRuntime());
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
