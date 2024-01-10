package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK4;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name = "Red Backstage Auto", group = "Competition", preselectTeleOp = "Main TeleOp")
public class RedBackStageAuto extends AutoBase {
    public static final Pose2d DROP_1 = new Pose2d(12, -37.5, Math.toRadians(90));
    public static final Pose2d DROP_2 = new Pose2d(12, -37.5, Math.toRadians(90));
    public static final Pose2d DROP_3 = new Pose2d(12, -37.5, Math.toRadians(90));
    public static final Pose2d DEPOSIT_PRELOAD_1 = new Pose2d(52.5, -32, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_2 = new Pose2d(52.5, -32, Math.toRadians(180));
    public static final Pose2d DEPOSIT_PRELOAD_3 = new Pose2d(52.5, -32, Math.toRadians(180));
    public static final Vector2d POST_SCORING_SPLINE_END = new Vector2d(12, -10.5);//-36
    public static final Pose2d STACK_LOCATION = new Pose2d(-56, -11, Math.toRadians(180));
    public Trajectory scorePurple1;
    public Trajectory scorePurple2;
    public Trajectory scorePurple3;

    public Trajectory scoreYellow1;
    public Trajectory scoreYellow2;
    public Trajectory scoreYellow3;

    public Trajectory parkRobot1;
    public Trajectory parkRobot2;
    public Trajectory parkRobot3;

    public Trajectory stackrun1b1;
    public Trajectory stackrun1b2;
    public Trajectory stackrun1b3;

    public Trajectory returnstackrun1b1;
    public Trajectory returnstackrun1b2;
    public Trajectory returnstackrun1b3;

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
                switch(teamPropLocation) {
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
            case 8:
                // if drive is done move on
                if (!robot.drive.isBusy()) {
                    macroTime = getRuntime();
                    macroState++;
                }
                break;
            // RUN INTAKE
            case 2:
                // intake
                if (getRuntime() < macroTime + 0.5) {
                    robot.intake.setDcMotor(-0.46);
                }
                // if intake is done move on
                else {
                    robot.intake.setDcMotor(0);
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                    builder = this.robot.getTrajectorySequenceBuilder();
                    switch(teamPropLocation) {
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
                    builder.splineToConstantHeading(POST_SCORING_SPLINE_END, 0);
                    builder.lineToConstantHeading(STACK_LOCATION.vec());
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            case 5:
                robot.resetMacro(0, getRuntime());
                if(!robot.drive.isBusy()){
                    macroState++;
                }
                break;
            case 6:
                robot.intake.setDcMotor(0.46);
                robot.intake.setpos(STACK5);
                macroTime = getRuntime();
                macroState ++;
                break;

            case 7:
                if (getRuntime() > macroTime + 1.5) {
                    builder = this.robot.getTrajectorySequenceBuilder();
                    builder.lineToConstantHeading(POST_SCORING_SPLINE_END);
                    switch(teamPropLocation) {
                        case 1:
                            builder.splineToConstantHeading(DEPOSIT_PRELOAD_1.vec(), 0);
                            break;
                        case 2:
                            builder.splineToConstantHeading(DEPOSIT_PRELOAD_2.vec(), 0);
                            break;
                        case 3:
                            builder.splineToConstantHeading(DEPOSIT_PRELOAD_3.vec(), 0);
                            break;
                    }
                    this.robot.drive.followTrajectorySequenceAsync(builder.build());
                    macroState++;
                }
                break;
            case 9:
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
            case 10:
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