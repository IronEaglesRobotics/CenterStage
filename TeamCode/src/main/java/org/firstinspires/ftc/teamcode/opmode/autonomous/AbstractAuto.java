package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

public abstract class AbstractAuto extends LinearOpMode {

    public Robot robot;
    private int teamElementLocation = 2;
    private double currentRuntime;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        makeTrajectories();

        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        while (!(isStarted() || isStopRequested())) {
            currentRuntime = getRuntime();
            robot.update(currentRuntime);

            int newLocation = robot.camera.getMarkerId();
            if (newLocation != -1) {
                teamElementLocation = newLocation;
            }

            telemetry.addLine("Initialized");
            telemetry.addLine("Randomization: " + teamElementLocation);
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }
    }

    public abstract void setAlliance();

    public abstract void makeTrajectories();

    public abstract void setCameraPosition();

    public abstract boolean useCamera();
}