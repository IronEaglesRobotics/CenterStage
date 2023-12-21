package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Red Right Auto")
public class RedRightAuto extends AbstractAuto {
    public Trajectory scorePurple;
    public Trajectory scoreYellow;
    public Trajectory load1;
    public Trajectory score1;
    public Trajectory load2;
    public Trajectory score2;
    public Trajectory parkRobot;

    @Override
    public void makeTrajectories() {
        Pose2d start = new Pose2d(12, -61.5, Math.toRadians(90));

        Pose2d dropMiddle = new Pose2d(12, -40.5, Math.toRadians(90));

        Pose2d depositPreload = new Pose2d(46, -36, Math.toRadians(180));

        Pose2d lineup1 = new Pose2d(12, -14, Math.toRadians(180));
        Pose2d pickup1 = new Pose2d(-58, -14, Math.toRadians(180));

        Pose2d driveup1 = new Pose2d(12-6, -14, Math.toRadians(180));
        Pose2d deposit1 = new Pose2d(48-6, -36-4, Math.toRadians(180));

        Pose2d lineup2 = new Pose2d(12, -14, Math.toRadians(180));
        Pose2d pickup2 = new Pose2d(-58, -14, Math.toRadians(180));

        Pose2d driveup2 = new Pose2d(12-6, -14, Math.toRadians(180));
        Pose2d deposit2 = new Pose2d(48-6, -36-4, Math.toRadians(180));

        Pose2d park = new Pose2d(48, -12, Math.toRadians(180));

        scorePurple = robot.drive.trajectoryBuilder(start)
                .lineToLinearHeading(dropMiddle)
                .build();

        scoreYellow = robot.drive.trajectoryBuilder(scorePurple.end())
                .lineToLinearHeading(depositPreload)
                .build();

        int driveSpeed = 45;

        load1 = robot.drive.trajectoryBuilder(scoreYellow.end())
                .splineToConstantHeading(lineup1.vec(), lineup1.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(pickup1,
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        score1 = robot.drive.trajectoryBuilder(load1.end(), true)
                .lineToLinearHeading(driveup1,
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(deposit1.vec(), deposit1.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        load2 = robot.drive.trajectoryBuilder(score1.end())
                .splineToConstantHeading(lineup1.vec(), lineup1.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(pickup1,
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        score2 = robot.drive.trajectoryBuilder(load2.end(), true)
                .lineToLinearHeading(driveup1,
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(deposit1.vec(), deposit1.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(driveSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        parkRobot = robot.drive.trajectoryBuilder(score2.end())
                .lineToLinearHeading(park)
                .build();

        robot.drive.setPoseEstimate(start);
    }

    @Override
    public void initializeSteps(int location) {
        // score preloads
        followTrajectory(scorePurple);
        runIntake(-0.4, 0.5);
        followAndExtend(scoreYellow, Slides.Position.TIER1);

        // cycle
        followAndRetract(load1, Slides.Position.DOWN);
        intakeStack(Intake.Position.STACK5, Intake.Position.STACK4);
        followAndExtend(score1, Slides.Position.TIER1);
        followAndRetract(load2, Slides.Position.DOWN);
        intakeStack(Intake.Position.STACK3, Intake.Position.STACK2);
        followAndExtend(score2, Slides.Position.TIER1);

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
}
