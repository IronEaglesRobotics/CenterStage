package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "BlueRightAuto(Drive only)")
public class BlueRightAuto_Drivepathonly_ extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//      add more trajectories here
        Pose2d start = new Pose2d(-24, 61.5, Math.toRadians(-90));

//        Pose2d dropLeft = new Pose2d(24, 60, Math.toRadians(-90));
        Pose2d dropMiddle = new Pose2d(-24, 40.5, Math.toRadians(-180));
//        Pose2d dropRight = new Pose2d(24, 60, Math.toRadians(-90));

        Pose2d stack1 = new Pose2d(-68, 40.5, Math.toRadians(-180));

        Pose2d dropMiddle2 = new Pose2d(-24, 40.5, Math.toRadians(-180));

        Pose2d bmid = new Pose2d(-24, 5, Math.toRadians(-180));

        Pose2d bmid2 = new Pose2d(24, 5, Math.toRadians(-180));

        Pose2d alimb = new Pose2d(60, 40.5, Math.toRadians(-180));

        Pose2d score = new Pose2d(60, 36, Math.toRadians(180));

        drive.setPoseEstimate(start);
//      add this per trajectories
        Trajectory scorePurple = drive.trajectoryBuilder(start)
                .lineToLinearHeading(dropMiddle)
                .build();

        Trajectory stack= drive.trajectoryBuilder(dropMiddle)
                .lineToLinearHeading(stack1)
                .build();

        Trajectory back_to_mid= drive.trajectoryBuilder(stack1)
                .lineToLinearHeading(dropMiddle2)
                .build();

        Trajectory front_gate= drive.trajectoryBuilder(dropMiddle2)
                .lineToLinearHeading(bmid)
                .build();

        Trajectory front_gate_pt_2= drive.trajectoryBuilder(bmid)
                .lineToLinearHeading(bmid2)
                .build();

        Trajectory about_to_score= drive.trajectoryBuilder(bmid2)


                .lineToLinearHeading(alimb)
                .build();

// the scorePurple. should be whatever the start pose2d thing was
        Trajectory scoreYellow = drive.trajectoryBuilder(scorePurple.end())
                .lineToLinearHeading(score)
                .build();

        waitForStart();

        if(isStopRequested()) return;
//      add this per trajectories
        drive.followTrajectory(scorePurple);
        drive.followTrajectory(stack);
        drive.followTrajectory(back_to_mid);
        drive.followTrajectory(front_gate);
        drive.followTrajectory(front_gate_pt_2);
        drive.followTrajectory(about_to_score);
        drive.followTrajectory(scoreYellow);
    }
}
