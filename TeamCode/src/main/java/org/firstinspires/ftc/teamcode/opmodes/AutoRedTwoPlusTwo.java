package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "autoRed2+2")
public class AutoRedTwoPlusTwo extends LinearOpMode {
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
    protected Trajectory boardThree;
    protected Trajectory backOffThree;


    protected Trajectory park1;
    protected Trajectory park2;

    protected Trajectory goGate;
    protected Trajectory goStack;
    protected Trajectory backGate;
    protected Trajectory approachBoard;
    protected Trajectory scoreStack;


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
                .lineToLinearHeading(new Pose2d(29.5, -29, Math.toRadians(360)))
                .build();

        this.boardOne = this.robot.getDrive().trajectoryBuilder(scoreOne.end())
                .lineToLinearHeading(new Pose2d(76, -26.5, Math.toRadians(360)))
                .addTemporalMarker(.2, robot.getArm()::armScore)
                .addTemporalMarker(.2,robot.getSlides()::slideFirstLayer)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffOne = this.robot.getDrive().trajectoryBuilder(boardOne.end())
                .lineToLinearHeading(new Pose2d(60, -15, Math.toRadians(360)))
                .build();


        //Randomization Two
        this.preloadTwo = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(33, -28, Math.toRadians(270)))
                .build();

        this.scoreTwo = this.robot.getDrive().trajectoryBuilder(preloadTwo.end())
                .lineToLinearHeading(new Pose2d(75.7, -36.3, Math.toRadians(360)))
                .addTemporalMarker(.2, robot.getArm()::armScore)
                .addTemporalMarker(.2,robot.getSlides()::slideFirstLayer)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffTwo = this.robot.getDrive().trajectoryBuilder(scoreTwo.end())
                .lineToLinearHeading(new Pose2d(60, -35, Math.toRadians(360)))
                .build();

        //Randomization Three
        this.preloadThree = this.robot.getDrive().trajectoryBuilder(initialPosition)
                .lineToLinearHeading(new Pose2d(46, -35, Math.toRadians(270)))
                .build();

        this.scoreThree = this.robot.getDrive().trajectoryBuilder(preloadThree.end())
                .lineToLinearHeading(new Pose2d(75, -42, Math.toRadians(360)))
                .addTemporalMarker(.2, robot.getArm()::armScore)
                .addTemporalMarker(.2,robot.getSlides()::slideFirstLayer)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .build();

        this.backOffThree = this.robot.getDrive().trajectoryBuilder(scoreThree.end())
                .lineToLinearHeading(new Pose2d(60, -45, Math.toRadians(360)))
                .build();

        //Park
        this.park1 = this.robot.getDrive().trajectoryBuilder(backOffTwo.end())
                .lineToLinearHeading(new Pose2d(65, -10, Math.toRadians(360)))
                .addTemporalMarker(.3, robot.getArm()::armRest)
                .addTemporalMarker(.3, robot.getWrist()::wristPickup)
                .addTemporalMarker(.1,robot.getSlides()::slideDown)
                .build();
        this.park2 = this.robot.getDrive().trajectoryBuilder(park1.end())
                .lineToLinearHeading(new Pose2d(80, -57, Math.toRadians(360)))
                .build();

//        //Cycle
        this.goGate = this.robot.getDrive().trajectoryBuilder(park1.end())
                .lineToLinearHeading(new Pose2d(-41,-8, Math.toRadians(360)))
                .addTemporalMarker(.5,robot.getClaw()::openStack)
                .addTemporalMarker(.2,robot.getArm()::armPickupStack)
                .addTemporalMarker(.1,robot.getSlides()::slideDown)
                .build();
        this.backGate = this.robot.getDrive().trajectoryBuilder(goGate.end())
                .lineToLinearHeading(new Pose2d(50,-10, Math.toRadians(360)))
                .build();
        this.approachBoard = this.robot.getDrive().trajectoryBuilder(backGate.end())
                .lineToLinearHeading(new Pose2d(73.5, -28, Math.toRadians(360)))
                .addTemporalMarker(.2, robot.getArm()::armSecondaryScore)
                .addTemporalMarker(.2, robot.getWrist()::wristScore)
                .addTemporalMarker(.2,robot.getSlides()::slideAutoStacks)
                .build();
        this.scoreStack = this.robot.getDrive().trajectoryBuilder(approachBoard.end())
                .lineToLinearHeading(new Pose2d(72.5, -28, Math.toRadians(360)))
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
                this.robot.getDrive().followTrajectory(boardOne);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffOne);
                this.robot.getSlides().slideDown();
                sleep(300);
                break;
            case "CENTER":
                this.robot.getDrive().followTrajectory(preloadTwo);
                this.robot.getDrive().followTrajectory(scoreTwo);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffTwo);
                this.robot.getSlides().slideDown();
                sleep(300);
                break;
            case "RIGHT":
                this.robot.getDrive().followTrajectory(preloadThree);
                this.robot.getDrive().followTrajectory(scoreThree);
                sleep(500);
                this.robot.getClaw().open();
                sleep(500);
                this.robot.getDrive().followTrajectory(backOffThree);
                this.robot.getSlides().slideDown();
                sleep(300);
                break;
        }
        //Cycle
        this.robot.getDrive().followTrajectory(park1);
        this.robot.getDrive().followTrajectory(goGate);
        sleep(500);
        this.robot.getClaw().close();
        sleep(250);
        this.robot.getDrive().followTrajectory(backGate);
        this.robot.getDrive().followTrajectory(approachBoard);
        this.robot.getClaw().setPos(.86);
        sleep(200);

        double currentPos = .86;
        double targetPos = .78;
        double delta = (targetPos - currentPos) / 100;
        for (int i = 0 ; i < 100; i++) {
            this.robot.getClaw().setPos(currentPos + (delta * i));
            sleep(12);
        }

        sleep(300);
        this.robot.getDrive().followTrajectory(park1);
    }

}
