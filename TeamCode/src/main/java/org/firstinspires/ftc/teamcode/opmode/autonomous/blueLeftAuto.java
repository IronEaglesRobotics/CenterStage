package org.firstinspires.ftc.teamcode.opmode.autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Blue Left", group = "Left Start", preselectTeleOp = "Khang Main")
public class blueLeftAuto {
    public SampleMecanumDrive drive;
    public Robot robot;
    public Camera camera;
    private boolean camEnabled = true;
    public CameraPosition cameraPosition;
    public Trajectory start;

    public void Robot(HardwareMap hardwareMap) {
        //set to new Drive to revert
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        camEnabled = true;
    }

    @Override
    public void makeTrajectories() {

        // positions
        Pose2d start = new Pose2d(-65.125,6,Math.toRadians(180));
//        Pose2d scoreSpikeLeft = new Pose2d(-36,6,Math.toRadians(-90));
//        Pose2d scoreSpikeCenter = new Pose2d(-36,6,Math.toRadians(180));
//        Pose2d scoreSpikeRight = new Pose2d(-36,6,Math.toRadians(90));
//        Pose2d scoreBoardLeft = new Pose2d(-42,48,Math.toRadians(-90));
//        Pose2d scoreBoardCenter = new Pose2d(-36,48,Math.toRadians(-90));
//        Pose2d scoreBoardRight = new Pose2d(-30,48,Math.toRadians(-90));
//        Pose2d park1 = new Pose2d(-60,48,Math.toRadians(-90));
//        Pose2d park2 = new Pose2d(-60,60,Math.toRadians(-90));

//        this.start = robot.drive.trajectoryBuilder(start)
//                .lineToLinearHeading(scoreSpikeCenter)
//                .build();
//        this.start = robot.drive.trajectoryBuilder(scoreSpikeCenter)
//                .lineToLinearHeading(scoreBoardCenter)
//                .build();
//        this.start = robot.drive.trajectoryBuilder(scoreBoardCenter)
//                .lineToLinearHeading(park1)
//                .build();
//        this.start = robot.drive.trajectoryBuilder(park1)
//                .lineToLinearHeading(park2)
//                .build();

        drive.setPoseEstimate(start);

        Trajectory scoreSpikeCenter = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-36,6), Math.toRadians(180))
                .build();

        Trajectory scoreBoardCenter = drive.trajectoryBuilder(scoreSpikeCenter.end())
                .splineTo(new Vector2d(-36,48), Math.toRadians(-90))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(scoreBoardCenter.end())
                .splineTo(new Vector2d(-60,48), Math.toRadians(-90))
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1.end())
                .splineTo(new Vector2d(-60,60), Math.toRadians(180))
                .build();

        drive.followTrajectory(scoreSpikeCenter);
        drive.followTrajectory(scoreBoardCenter);
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);
    }
}

