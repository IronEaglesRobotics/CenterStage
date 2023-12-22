package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public abstract class AutoBase extends LinearOpMode {
    public Robot robot;

    protected int macroState;
    protected double macroTime;
    protected int teamPropLocation = 2; // 1 is left, 2 is middle, 3 is right, perhaps these could be enums instead

    // abstract methods for each auto to implement
    public abstract void createTrajectories();
    public abstract void followTrajectories();

    @Override
    public void runOpMode() {
        // create telemetry object for both driver hub and dashboard
        // check SampleMecanumDrive to make sure it doesn't override the dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // initialize robot
        robot = new Robot(hardwareMap);

        // create trajectories
        createTrajectories();

        // wait for start
        while (!isStarted() && !isStopRequested()) {
            if (robot.camera.getFrameCount() < 1) {
                telemetry.addLine("Initializing...");
            } else {
                telemetry.addLine("Initialized");
                //teamPropLocation = robot.camera.getMarkerId(); // or whatever method you end up using
                telemetry.addData("Team Prop Location", teamPropLocation);
            }
            telemetry.update();
        }

        // run auto
        while (macroState != -1 && !isStopRequested()) {
            // state machine to follow trajectories
            followTrajectories();

            // update robot
            robot.update(getRuntime());

            // update telemetry
            telemetry.addData("Macro State", macroState);
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }

        // stop camera
        robot.camera.stopTargetingCamera();
    }
}
