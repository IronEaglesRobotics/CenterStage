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
@Autonomous(name = "Blue FrontPreload (2+1)", group = "Competition", preselectTeleOp = "Main TeleOp")
public class BlueFrontPreload extends AutoBase {
    public static final Pose2d DROP_1_PART_2 = new Pose2d(-33, 33.5, Math.toRadians(0));

    public static final Pose2d DROP_1 = new Pose2d(-33,33.5,Math.toRadians(0));
    public static final Pose2d DROP_2 = new Pose2d(-39.7, 33.5, Math.toRadians(-90));
    public static final Pose2d DROP_2_PART_2 = new Pose2d(-39.7,36.5, Math.toRadians(-90));
    public static final Pose2d DROP_3 = new Pose2d(-51, 50.5, Math.toRadians(-90));
    public static final Pose2d DROP_1M = new Pose2d(-36, 45, Math.toRadians(-90));

    public static final Pose2d DROP_2M = new Pose2d(-48.5, 30, Math.toRadians(-90));

    public static final Pose2d DROP_3M = new Pose2d(-53.7, 35.9, Math.toRadians(-90));

    public static final Pose2d DEPOSIT_WHITE_STACKS_1 = new Pose2d(53.3, 38.3, Math.toRadians(0));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_2 = new Pose2d(53, 34.5, Math.toRadians(0));//187

    public static final Pose2d DEPOSIT_WHITE_STACKS_3 = new Pose2d(55, 25, Math.toRadians(0));//817

    public static final Pose2d STACK_LOCATION_1 = new Pose2d(-54.5, 32.6, Math.toRadians(180));

    public static final Pose2d STACK_LOCATION_2 = new Pose2d(-52, 29.6, Math.toRadians(180));

    public static final Pose2d STACK_LOCATION_3 = new Pose2d(-52, 29.6, Math.toRadians(180));

    public static final Pose2d POST_SCORING_SPLINE_END = new Pose2d(24, 12.4, Math.toRadians(10));//-36

    public static final Pose2d POST_DROP_POS = new Pose2d(-45, 59.5, Math.toRadians(180));

    public static final Pose2d POST_DROP_POS_PART2 = new Pose2d(-33, 58.5, Math.toRadians(180));

    public static final Pose2d PRE_DEPOSIT_POS = new Pose2d(33, 58.5, Math.toRadians(180));

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
                    switch (teamPropLocation) {
                        case (1):
                            //team prop location 1
                            builder.lineToLinearHeading(STACK_LOCATION_1.plus(new Pose2d(0))).waitSeconds(.01);
                            break;
                        case (2):
                            //team prop location 2
                            builder.lineToLinearHeading(STACK_LOCATION_2.plus(new Pose2d(-5.8,2))).waitSeconds(.01);
                            break;
                        case (3):
                            //team prop location 3
                            builder.lineToLinearHeading(STACK_LOCATION_3.plus(new Pose2d(-4.8,2))).waitSeconds(.01);
                            break;
                    }


                    robot.intake.setDcMotor(0.44);
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
                if (getRuntime() > macroTime + 2.4 || !robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    robot.macroState = 0;
                    robot.extendMacro(Slides.mini_tier1-30, getRuntime());
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

            //Park

            case 7:
                robot.resetMacro(0, getRuntime());
                if (getRuntime() > macroTime + 3.4 || robot.macroState >= 4) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToLinearHeading(PARK_1);
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;

            case 8:
                // if drive is done move on
                if (getRuntime() > macroTime + 4.4 || !robot.drive.isBusy()) {
                    macroState++;
                }
                break;

            case 9:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState == 0) {
                    macroState = -1;
                }

            case 19:
                robot.resetMacro(0, getRuntime());
                if (robot.macroState == 0) {
                    macroState = -1;
                }
        }
    }
}