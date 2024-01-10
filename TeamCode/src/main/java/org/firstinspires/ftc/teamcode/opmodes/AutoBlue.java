package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "autoBlue")
public class AutoBlue extends LinearOpMode {
    protected Pose2d initialPosition;

    protected Trajectory preloadOne;
    protected Trajectory scoreOne;
    protected Trajectory boardOne;
    protected Trajectory backOffOne;


    protected Trajectory preloadTwo;
    protected Trajectory scoreTwo;
    protected Trajectory backOffTwo;


    protected Trajectory preloadThree;
    protected Trajectory scoreThree;
    protected Trajectory backOffThree;


    protected Trajectory park1;
    protected Trajectory park2;


    private Robot robot;
    private String randomization;
    private int random;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.robot.getCamera().initTargetingCamera();

        //Trajectories
        this.initialPosition = new Pose2d(-34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        //Randomization One
        this.preloadOne = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(-40, -37.5, Math.toRadians(270)))
                .build();

        this.scoreOne = this.robot.getDrive().trajectoryBuilder(preloadOne.end())
                .lineToLinearHeading(new Pose2d(-29, -32, Math.toRadians(180)))
                .build();

        this.boardOne = this.robot.getDrive().trajectoryBuilder(scoreOne.end())
                .lineToLinearHeading(new Pose2d(-72, -26.3, Math.toRadians(180)))
                .addTemporalMarker(.2, robot.getArm()::armAccurateScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffOne = this.robot.getDrive().trajectoryBuilder(boardOne.end())
                .lineToLinearHeading(new Pose2d(-60, -26, Math.toRadians(180)))
                .build();


        //Randomization Two
        this.preloadTwo = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(-35, -28, Math.toRadians(270)))
                .build();

        this.scoreTwo = this.robot.getDrive().trajectoryBuilder(preloadTwo.end())
                .lineToLinearHeading(new Pose2d(-70, -34, Math.toRadians(180)))
                .addTemporalMarker(.2, robot.getArm()::armAccurateScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffTwo = this.robot.getDrive().trajectoryBuilder(scoreTwo.end())
                .lineToLinearHeading(new Pose2d(-60, -34, Math.toRadians(180)))
                .build();

        //Randomization Three
        this.preloadThree = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(-42, -35, Math.toRadians(270)))
                .build();

        this.scoreThree = this.robot.getDrive().trajectoryBuilder(preloadThree.end())
                .lineToLinearHeading(new Pose2d(-71.5, -41.3, Math.toRadians(180)))
                .addTemporalMarker(.2, robot.getArm()::armAccurateScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffThree = this.robot.getDrive().trajectoryBuilder(scoreThree.end())
                .lineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(180)))
                .build();

        //Park
        this.park1 = this.robot.getDrive().trajectoryBuilder(backOffTwo.end())
                .lineToLinearHeading(new Pose2d(-65, -55, Math.toRadians(180)))
                .addTemporalMarker(.3, robot.getArm()::armRest)
                .addTemporalMarker(.3, robot.getWrist()::wristPickup)
                .build();
        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
                .lineToLinearHeading(new Pose2d(-80, -60, Math.toRadians(180)))
                .build();

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPositionBlue());
            randomization = String.valueOf(this.robot.getCamera().getStartingPositionBlue());
            this.telemetry.update();
        }

        switch (randomization) {
            case "RIGHT":
                this.robot.getDrive().followTrajectory(preloadOne);
                this.robot.getDrive().followTrajectory(scoreOne);
                this.robot.getDrive().followTrajectory(boardOne);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffOne);
                sleep(500);
                this.robot.getDrive().followTrajectory(park1);
                this.robot.getDrive().followTrajectory(park2);
                break;
            case "CENTER":
                this.robot.getDrive().followTrajectory(preloadTwo);
                this.robot.getDrive().followTrajectory(scoreTwo);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffTwo);
                sleep(500);
                this.robot.getDrive().followTrajectory(park1);
                this.robot.getDrive().followTrajectory(park2);
                break;
            case "LEFT":
                this.robot.getDrive().followTrajectory(preloadThree);
                this.robot.getDrive().followTrajectory(scoreThree);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffThree);
                sleep(500);
                this.robot.getDrive().followTrajectory(park1);
                this.robot.getDrive().followTrajectory(park2);
                break;
        }


    }

}
