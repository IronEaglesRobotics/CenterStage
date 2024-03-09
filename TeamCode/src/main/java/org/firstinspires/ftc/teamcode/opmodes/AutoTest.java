package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Test Stuff")
public class AutoTest extends LinearOpMode {
    protected Pose2d initialPosition;
    private Robot robot;

    //Pose2ds
    //Preloads
    final static Pose2d FIRSTMOVE = new Pose2d(10, 10, Math.toRadians(0));

    protected void sequenceOne() {
        TrajectorySequenceBuilder builder = this.robot.getTrajectorySequenceBuilder();
        builder.splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(0));
        this.robot.getDrive().followTrajectorySequence(builder.build());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.initialPosition = new Pose2d(0, 0, Math.toRadians(0));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        // Do super fancy chinese shit
        while (!this.isStarted()) {
        }

        sequenceOne();

    }

}
