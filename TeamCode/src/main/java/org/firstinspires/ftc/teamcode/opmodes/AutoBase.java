package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.AutoRedFarTwoPlusTwo.autoState;

public abstract class AutoBase extends LinearOpMode {
    protected Pose2d initialPosition;
    Robot robot;
    String randomization;
    String parkLocation;
    double runtime;

    autoState state = autoState.PURPLE;
    int delay;

    public abstract void followTrajectories();

    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot().init(hardwareMap);
        this.initialPosition = new Pose2d(-34, -59.5, Math.toRadians(270));
        this.robot.getDrive().setPoseEstimate(initialPosition);

        while (!this.isStarted()) {
            parkLocation = "RIGHT";
            this.telemetry.addData("Starting Position", this.robot.getCamera().getStartingPosition());
            randomization = String.valueOf(this.robot.getCamera().getStartingPosition());
            this.telemetry.addData("Park Position", parkLocation);
            this.telemetry.addData("Delay", delay);
            this.telemetry.update();
        }

        while (state != autoState.STOP) {
            followTrajectories();
            robot.update();
        }
    }
}
