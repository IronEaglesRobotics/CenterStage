package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "autoBlueFar")
public class AutoBlueFar extends LinearOpMode {
    protected Pose2d initialPosition;

    protected Trajectory preloadOne;
    protected Trajectory scoreOne;
    protected Trajectory boardOne;
    protected Trajectory backOffOne;
    protected Trajectory goGate1;
    protected Trajectory passGate;

    protected Trajectory preloadTwo;
    protected Trajectory scoreTwo;
    protected Trajectory backOffTwo;
    protected Trajectory tokyoDrift;
    protected Trajectory tokyoDrift2;
    protected Trajectory tokyoDrift3;


    protected Trajectory preloadThree;
    protected Trajectory boardThree;
    protected Trajectory scoreThree;
    protected Trajectory backOffThree;
    protected Trajectory goGate3;
    protected Trajectory goGate3Again;


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
        this.initialPosition = new Pose2d(34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        //Randomization One
        this.preloadOne = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(40, -37.5, Math.toRadians(270)))
                .build();

        this.scoreOne = this.robot.getDrive().trajectoryBuilder(preloadOne.end())
                .lineToLinearHeading(new Pose2d(35, -20, Math.toRadians(150)))
                .build();

        this.goGate1 = this.robot.getDrive().trajectoryBuilder(scoreOne.end())
                .lineToLinearHeading(new Pose2d(31, -10, Math.toRadians(180)))
                .build();

        this.passGate = this.robot.getDrive().trajectoryBuilder(goGate1.end())
                .lineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)))
                .build();

        this.boardOne = this.robot.getDrive().trajectoryBuilder(passGate.end())
                .lineToLinearHeading(new Pose2d(-50, -28, Math.toRadians(180)))
                .addTemporalMarker(.2, robot.getArm()::armSecondaryScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffOne = this.robot.getDrive().trajectoryBuilder(boardOne.end())
                .lineToLinearHeading(new Pose2d(-40, -25, Math.toRadians(180)))
                .build();

        //Randomization Two
        this.preloadTwo = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(36, -28, Math.toRadians(290)))
                .build();

        this.tokyoDrift = this.robot.getDrive().trajectoryBuilder(preloadTwo.end())
                .lineToLinearHeading(new Pose2d(50, -38, Math.toRadians(270)))
                .build();

        this.tokyoDrift2 = this.robot.getDrive().trajectoryBuilder(tokyoDrift.end())
                .lineToLinearHeading(new Pose2d(50, -9, Math.toRadians(180)))
                .build();

        this.tokyoDrift3 = this.robot.getDrive().trajectoryBuilder(tokyoDrift2.end())
                .lineToLinearHeading(new Pose2d(35, -9, Math.toRadians(180)))
                .build();

        this.scoreTwo = this.robot.getDrive().trajectoryBuilder(passGate.end())
                .lineToLinearHeading(new Pose2d(-50, -33, Math.toRadians(180)))
                .addTemporalMarker(.02, robot.getArm()::armSecondaryScore)
                .addTemporalMarker(.02, robot.getWrist()::wristScore)
                .build();

        this.backOffTwo = this.robot.getDrive().trajectoryBuilder(scoreTwo.end())
                .lineToLinearHeading(new Pose2d(-40, -33, Math.toRadians(180)))
                .build();

        //Randomization Three
        this.preloadThree = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(43, -37.5, Math.toRadians(270)))
                .build();

        this.scoreThree = this.robot.getDrive().trajectoryBuilder(preloadThree.end())
                .lineToLinearHeading(new Pose2d(29, -32, Math.toRadians(335)))
                .build();

        this.goGate3 = this.robot.getDrive().trajectoryBuilder(scoreThree.end())
                .lineToLinearHeading(new Pose2d(40, -32, Math.toRadians(335)))
                .build();

        this.goGate3Again = this.robot.getDrive().trajectoryBuilder(goGate3.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(180)))
                .build();

        this.boardThree = this.robot.getDrive().trajectoryBuilder(passGate.end())
                .lineToLinearHeading(new Pose2d(-50.5, -39, Math.toRadians(180)))
                .addTemporalMarker(.2, robot.getArm()::armSecondaryScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffThree = this.robot.getDrive().trajectoryBuilder(boardThree.end())
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(180)))
                .build();


        //Park
        this.park1 = this.robot.getDrive().trajectoryBuilder(backOffOne.end())
                .lineToLinearHeading(new Pose2d(-40, -10, Math.toRadians(180)))
                .addTemporalMarker(.3, robot.getArm()::armRest)
                .addTemporalMarker(.3, robot.getWrist()::wristPickup)
                .build();
        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
                .lineToLinearHeading(new Pose2d(-60, -10, Math.toRadians(170)))
                .build();

        // Do super fancy chinese shit
        while (!this.isStarted()) {
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPositionBlue());
            randomization = String.valueOf(this.robot.getCamera().getStartingPositionBlue());
            this.telemetry.update();
        }

        sleep(5000);

        switch (randomization) {
            case "RIGHT":
                this.robot.getDrive().followTrajectory(preloadOne);
                this.robot.getDrive().followTrajectory(scoreOne);
                this.robot.getDrive().followTrajectory(goGate1);
                this.robot.getDrive().followTrajectory(passGate);
                this.robot.getDrive().followTrajectory(boardOne);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffOne);
                sleep(300);
                break;
            case "CENTER":
                this.robot.getDrive().followTrajectory(preloadTwo);
                this.robot.getDrive().followTrajectory(tokyoDrift);
                this.robot.getDrive().followTrajectory(tokyoDrift2);
                this.robot.getDrive().followTrajectory(passGate);
                this.robot.getDrive().followTrajectory(scoreTwo);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffTwo);
                sleep(300);
                break;
            case "LEFT":
                this.robot.getDrive().followTrajectory(preloadThree);
                this.robot.getDrive().followTrajectory(scoreThree);
                this.robot.getDrive().followTrajectory(goGate3);
                this.robot.getDrive().followTrajectory(goGate3Again);
                this.robot.getDrive().followTrajectory(passGate);
                this.robot.getDrive().followTrajectory(boardThree);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffThree);
                sleep(300);
                break;
        }
        //Cycle
        this.robot.getDrive().followTrajectory(park1);
        this.robot.getDrive().followTrajectory(park2);


    }

}
