package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Simple Blue Left Auto")
public class SimpleBlueLeftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(24, 61.5, Math.toRadians(-90));

//        Pose2d dropLeft = new Pose2d(24, 60, Math.toRadians(-90));
        Pose2d dropMiddle = new Pose2d(24, 40.5, Math.toRadians(-90));
//        Pose2d dropRight = new Pose2d(24, 60, Math.toRadians(-90));

        Pose2d score = new Pose2d(60, 36, Math.toRadians(180));

        drive.setPoseEstimate(start);

        Trajectory scorePurple = drive.trajectoryBuilder(start)
                .lineToLinearHeading(dropMiddle)
                .build();

        Trajectory scoreYellow = drive.trajectoryBuilder(scorePurple.end())
                .lineToLinearHeading(score)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(scorePurple);
//        drive.wait(1);
        drive.followTrajectory(scoreYellow);
    }
}
