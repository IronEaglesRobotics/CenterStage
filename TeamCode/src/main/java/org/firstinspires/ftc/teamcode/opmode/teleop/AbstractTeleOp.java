package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
public abstract class AbstractTeleOp extends OpMode {
    private Robot robot;
    Controller driver1;
    Controller driver2;

    public static double drivebaseThrottle = 0.6;
    public static double drivebaseTurbo = 1.0;
    public static int heightIncrement = 20;

    Pose2d robot_pos;
    double robot_x, robot_y, robot_heading;

    // auto align variables
    double headingPID;
    public static double headingP = 0.01;
    public static double headingI = 0.03;
    public static double headingD = 0.0005;

    double strafePID;
    public static double strafeP = 0.05;
    public static double strafeI = 0;
    public static double strafeD = 0.01;

    double robot_y_pos;
    boolean fixed90Toggle = false;
    boolean fixed0Toggle = false;

//    public static double robot_width = 12;
//    public static double robot_length = 12;
//    public static double robot_radius = 6;
//
//    public static double groundJuncRadius = 6;
//    public static double coneRadius = 4;
//
//    public static double armWait = 0.2;

//    private double timeSinceOpened = 0; // for claw
//    private double timeSinceClosed = 0;

//    private int delayState = 0; // for arm
//    private double delayStart = 0;
//    private boolean doArmDelay = false; // for arm
    private boolean isAutoClose = true;

    @Override
    public void init() {
        robot =  new Robot(hardwareMap);
        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
//        if (robot.camera.getFrameCount() < 1) {
//            telemetry.addLine("Initializing Robot...");
//        } else {
//            telemetry.addLine("Initialized");
//        }
        telemetry.addLine("Initialized");
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }

    private int getQuadrant(double angle) {
        if (0 < angle && angle < PI/2.0) {
            return 1;
        } else if (PI/2.0 < angle && angle < PI) {
            return 2;
        } else if (PI < angle && angle < 3*PI/2.0) {
            return 3;
        } else {
            return 4;
        }
    }

    @Override
    public void loop() {
        // robot position update
        //robot_pos = robot.drive.getPoseEstimate();
        //robot_x = robot_pos.getX();
        //robot_y = robot_pos.getY();
        //robot_heading = robot_pos.getHeading(); // in radians

        // driver 1 controls
        //driver1.update();
        //driver2.update();
        double x = -driver1.getLeftStick().getY();
        double y = driver1.getLeftStick().getX();
        double z = -driver1.getRightStick().getX();

//         figure out if a toggle is present
        if (driver1.getRightBumper().isPressed()) {
            if (!fixed90Toggle) {
                //robot_y_pos = robot.drive.getPoseEstimate().getX();
            }
            fixed90Toggle = true;
        } else {
            fixed90Toggle = false;
        }

        if (!fixed90Toggle && driver1.getLeftBumper().isPressed()) {
            if (!fixed0Toggle) {
                //robot_y_pos = robot.drive.getPoseEstimate().getY();
            }
            fixed0Toggle = true;
        } else {
            fixed0Toggle = false;
        }

        telemetry.addLine("Wanted Position: "+robot_y_pos);
        telemetry.addLine("Actual Position: "+robot_y);
        telemetry.addLine("PID: "+strafePID);

        // turbo
        if (driver1.getLeftBumper().isPressed() || driver1.getRightBumper().isPressed()) {
            x *= drivebaseTurbo;
            y *= drivebaseTurbo;
            z *= drivebaseTurbo;
        } else {
            x *= drivebaseThrottle;
            y *= drivebaseThrottle;
            z *= drivebaseThrottle;
        }

        // actually set power
        if (fixed0Toggle || fixed90Toggle) {
            //robot.drive.setWeightedDrivePower(new Pose2d(x, 0, headingPID));
        } else {
            //robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        }

        // update and telemetry
        //robot.update(getRuntime());

        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
