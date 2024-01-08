package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK4;
import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.STACK5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

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

        // create pose2d variables
        // you might not need 3 instances of the deposit position, for example, however based on localization accuracy
        // you might need them for each one to be slightly different
        Pose2d drop1 = new Pose2d(12, -37.5, Math.toRadians(90));
        Pose2d drop2 = new Pose2d(12, -37.5, Math.toRadians(90));
        Pose2d drop3 = new Pose2d(12, -37.5, Math.toRadians(90));

        Pose2d depositPreload1 = new Pose2d(52.5, -32, Math.toRadians(180));
        Pose2d depositPreload2 = new Pose2d(52.5, -32, Math.toRadians(180));
        Pose2d depositPreload3 = new Pose2d(52.5, -32, Math.toRadians(180));

//        Pose2d park1 = new Pose2d(48, -12, Math.toRadians(180));
//        Pose2d park2 = new Pose2d(48, -12, Math.toRadians(180));
//        Pose2d park3 = new Pose2d(48, -12, Math.toRadians(180));
//
//        Pose2d toStack = new Pose2d(40,-36, Math.toRadians(180));

        Pose2d stack_1x1 = new Pose2d(-56, -11, Math.toRadians(180));//-36
        Pose2d stack_2x1 = new Pose2d(-56, -11, Math.toRadians(180));
        Pose2d stack_3x1 = new Pose2d(-56, -11, Math.toRadians(180));

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

//        parkRobot1 = robot.drive.trajectoryBuilder(scoreYellow1.end())
//                .lineToLinearHeading(park1)
//                .build();
//        parkRobot2 = robot.drive.trajectoryBuilder(scoreYellow2.end())
//                .lineToLinearHeading(park2)
//                .build();++

//        parkRobot3 = robot.drive.trajectoryBuilder(scoreYellow3.end())
//                .lineToLinearHeading(park3)
//                .build();

        stackrun1b1 = robot.drive.trajectoryBuilder(scoreYellow1.end())
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
                .lineToConstantHeading(stack_1x1.vec())
                .build();
        stackrun1b2 = robot.drive.trajectoryBuilder(scoreYellow2.end())
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
                .lineToLinearHeading(stack_2x1)
                .build();
        stackrun1b3 = robot.drive.trajectoryBuilder(scoreYellow3.end())
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
                .lineToLinearHeading(stack_3x1)
                .build();

        returnstackrun1b1 = robot.drive.trajectoryBuilder(stackrun1b1.end())
                .lineToLinearHeading(stack_1x1)
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
                .build();
        returnstackrun1b2 = robot.drive.trajectoryBuilder(stackrun1b2.end())
                .lineToLinearHeading(stack_2x1)
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
                .build();
        returnstackrun1b3 = robot.drive.trajectoryBuilder(stackrun1b3.end())
                .lineToLinearHeading(stack_3x1)
                .splineToConstantHeading(new Vector2d(12, -10.5), Math.toRadians(180))
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
                    robot.intake.setDcMotor(-0.46);
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
                if (robot.macroState >= 4){
                    robot.drive.followTrajectoryAsync(teamPropLocation==1?stackrun1b1:(teamPropLocation==2?stackrun1b2:stackrun1b3));
                    macroState++;
                }
                break;
            case 5:
                robot.resetMacro(0, getRuntime());
                if(!robot.drive.isBusy()){
                    macroState ++;
                }
                //macroState++;
                break;
            case 6:

                robot.intake.setDcMotor(0.46);
                robot.intake.setpos(STACK5);
                macroTime = getRuntime();
                macroState ++;
                break;

            case 7:
                if (getRuntime() > macroTime + 1.5) {
                robot.drive.followTrajectoryAsync(returnstackrun1b1);
                    macroState++;
                }
                break;

            case 8:
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