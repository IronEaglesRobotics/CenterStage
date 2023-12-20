package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Slides;

@Autonomous(name = "Blue Left Auto")
public class BlueLeftAuto extends AbstractAuto {
    public Trajectory scorePurple;
    public Trajectory scoreYellow;
    public Trajectory parkRobot;

    @Override
    public void initializeSteps(int location) {
       followTrajectory(scorePurple);
       runIntake(-0.3, 1);
       followAndExtend(scoreYellow, Slides.Position.TIER1);
       followAndRetract(parkRobot, Slides.Position.DOWN);
    }

    @Override
    public void setAlliance() {
    }

    @Override
    public void setCameraPosition() {
    }

    @Override
    public boolean useCamera() {
        return false;
    }

    @Override
    public void makeTrajectories() {
        Pose2d start = new Pose2d(24, 61.5, Math.toRadians(-90));
//        Pose2d dropLeft = new Pose2d(24, 60, Math.toRadians(-90));
        Pose2d dropMiddle = new Pose2d(24, 40.5, Math.toRadians(-90));
//        Pose2d dropRight = new Pose2d(24, 60, Math.toRadians(-90));
        Pose2d score = new Pose2d(62, 36, Math.toRadians(180));
        Pose2d park = new Pose2d(62, 60, Math.toRadians(180));

        scorePurple = robot.drive.trajectoryBuilder(start)
                .lineToLinearHeading(dropMiddle)
                .build();

        scoreYellow = robot.drive.trajectoryBuilder(scorePurple.end())
                .lineToLinearHeading(score)
                .build();

        parkRobot = robot.drive.trajectoryBuilder(scoreYellow.end())
                .lineToLinearHeading(park)
                .build();

        robot.drive.setPoseEstimate(start);
    }
}
