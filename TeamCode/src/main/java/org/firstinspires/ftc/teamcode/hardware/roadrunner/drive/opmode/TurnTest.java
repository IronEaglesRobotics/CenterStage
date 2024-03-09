package org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double DISTANCE = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        GamepadEx controller = new GamepadEx(this.gamepad1);
        boolean zig = false;
        while (opModeIsActive() && !isStopRequested()) {
            controller.readButtons();
            if (controller.wasJustPressed(GamepadKeys.Button.A)) {
                drive.turn(Math.toRadians(ANGLE));
            }

            if (controller.wasJustPressed(GamepadKeys.Button.B)) {
                TrajectoryBuilder builder = drive.trajectoryBuilder(drive.getPoseEstimate());
                builder.forward(DISTANCE);
                drive.followTrajectory(builder.build());
            }
            if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
                TrajectoryBuilder builder = drive.trajectoryBuilder(drive.getPoseEstimate());
                builder.back(DISTANCE);
                drive.followTrajectory(builder.build());
            }
        }
    }
}
