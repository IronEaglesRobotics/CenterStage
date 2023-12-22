package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Slides;

@Autonomous(name = "Red Backstage Auto", group = "Competition", preselectTeleOp = "Main TeleOp")
public class RedBackStageAuto extends AutoBase {
    public Trajectory scorePurple1;
    public Trajectory scorePurple2;
    public Trajectory scorePurple3;

    public Trajectory scoreYellow1;
    public Trajectory scoreYellow2;
    public Trajectory scoreYellow3;

    public Trajectory parkRobot1;
    public Trajectory parkRobot2;
    public Trajectory parkRobot3;

    @Override
    public void createTrajectories() {
        // set start position
        Pose2d start = new Pose2d(12, -61.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(start);

        // create pose2d variables
        // you might not need 3 instances of the deposit position, for example, however based on localization accuracy
        // you might need them for each one to be slightly different
        Pose2d drop1 = new Pose2d(12, -39.5, Math.toRadians(90));
        Pose2d drop2 = new Pose2d(12, -39.5, Math.toRadians(90));
        Pose2d drop3 = new Pose2d(12, -39.5, Math.toRadians(90));

        Pose2d depositPreload1 = new Pose2d(50.5, -32, Math.toRadians(187));
        Pose2d depositPreload2 = new Pose2d(50.5, -32, Math.toRadians(187));
        Pose2d depositPreload3 = new Pose2d(50.5, -32, Math.toRadians(187));

        Pose2d park1 = new Pose2d(48, -12, Math.toRadians(180));
        Pose2d park2 = new Pose2d(48, -12, Math.toRadians(180));
        Pose2d park3 = new Pose2d(48, -12, Math.toRadians(180));

        // create trajectories
        scorePurple1 = robot.drive.trajectoryBuilder(start)
                .lineToLinearHeading(drop1)
                .build();
        scorePurple2 = robot.drive.trajectoryBuilder(start)
                .lineToLinearHeading(drop2)
                .build();
        scorePurple3 = robot.drive.trajectoryBuilder(start)
                .lineToLinearHeading(drop3)
                .build();

        scoreYellow1 = robot.drive.trajectoryBuilder(scorePurple1.end())
                .lineToLinearHeading(depositPreload1)
                .build();
        scoreYellow2 = robot.drive.trajectoryBuilder(scorePurple2.end())
                .lineToLinearHeading(depositPreload2)
                .build();
        scoreYellow3 = robot.drive.trajectoryBuilder(scorePurple3.end())
                .lineToLinearHeading(depositPreload3)
                .build();

        parkRobot1 = robot.drive.trajectoryBuilder(scoreYellow1.end())
                .lineToLinearHeading(park1)
                .build();
        parkRobot2 = robot.drive.trajectoryBuilder(scoreYellow2.end())
                .lineToLinearHeading(park2)
                .build();
        parkRobot3 = robot.drive.trajectoryBuilder(scoreYellow3.end())
                .lineToLinearHeading(park3)
                .build();
    }

    @Override
    public void followTrajectories() {
        switch (macroState) {
            case 0:
                robot.drive.followTrajectoryAsync(teamPropLocation==1?scorePurple1:(teamPropLocation==2?scorePurple2:scorePurple3));
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
                if (getRuntime() < macroTime + 0.5) {
                    robot.intake.setDcMotor(-0.26);
                }
                // if intake is done move on
                else {
                    robot.intake.setDcMotor(0);
                    robot.extendMacro(Slides.mini_tier1, getRuntime());
                    robot.drive.followTrajectoryAsync(teamPropLocation==1?scoreYellow1:(teamPropLocation==2?scoreYellow2:scoreYellow3));
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
                if (robot.macroState >= 2){
                    robot.drive.followTrajectoryAsync(teamPropLocation==1?parkRobot1:(teamPropLocation==2?parkRobot2:parkRobot3));
                    macroState++;
                }
                break;
            // PARK ROBOT
            case 5:
                // reset macro'
                if (robot.macroState != 0) {
                    robot.resetMacro(0, getRuntime());
                }
                // if macro and drive are done, end auto
                if (robot.macroState == 0 && !robot.drive.isBusy()) {
                    macroState=-1;
                }
                break;
        }
    }
}