package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "ThisIsTheLongestFlippingOpModeNameEverLolIamSeahorse!")
public class Auto extends LinearOpMode {
    private Robot robot;
    private String randomization;
    private int random;
    protected Pose2d initialPosition;
    protected Trajectory preloadOne;
    protected Trajectory scoreOne;
    protected Trajectory preloadTwo;
    protected Trajectory preloadThree;


    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.robot.getCamera().initTargetingCamera();

    //Trajectories
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        //Preload One
            this.preloadOne = this.robot.getDrive().trajectoryBuilder(initialPosition)
                    .lineToLinearHeading(new Pose2d(40, -37.5, Math.toRadians(270)))
                    .build();

        this.scoreOne = this.robot.getDrive().trajectoryBuilder(preloadOne.end())
                .lineToLinearHeading(new Pose2d(33,-30, Math.toRadians(360)))
                .build();

        //Preload Two
        this.preloadTwo = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(40,-25, Math.toRadians(270)))
                .build();

        //Preload Three
        this.preloadThree = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(47.5,-37.5, Math.toRadians(270)))
                .build();

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPosition());
            randomization = String.valueOf(this.robot.getCamera().getStartingPosition());
            this.telemetry.update();
        }

        switch (randomization) {
            case "LEFT":
                this.robot.getDrive().followTrajectory(preloadOne);
                this.robot.getDrive().followTrajectory(scoreOne);
                break;
            case "CENTER":
                this.robot.getDrive().followTrajectory(preloadTwo);
                break;
            case "RIGHT":
                this.robot.getDrive().followTrajectory(preloadThree);
                break;
        }



        }

    }
