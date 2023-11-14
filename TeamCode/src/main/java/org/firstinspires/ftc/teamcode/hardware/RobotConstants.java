package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME = "backLeft";
    public static final String BACK_RIGHT_NAME = "backRight";
    public static final String LEFT_SLIDE_MOTOR_NAME = "slideLeft";
    public static final String RIGHT_SLIDE_MOTOR_NAME = "slideRight";
    public static final String CLAW_ARM_LEFT_NAME = "clawArmLeft";
    public static final String CLAW_ARM_RIGHT_NAME = "clawArmRight";
    public static final String CLAW_NAME = "claw";
    public static final String GANTRY_X_NAME = "gantry_x";
    public static final String GANTRY_ARM_NAME = "gantryArm";
    public static final String GANTRY_SCREW_NAME = "screw";
    public static final String ROBOT_LIFT_ROTATION_NAME = "liftRotation";
    public static final String ROBOT_LIFT_LIFT_NAME = "liftLift";
    public static final String WEBCAM_NAME = "webcam";
    public static final String PARALLEL_DRIVE_ODOMETRY = BACK_RIGHT_NAME;
    public static final String PERPENDICULAR_DRIVE_ODOMETRY = FRONT_RIGHT_NAME;

    // Drive
    public static double SLOW_MODE_SPEED_PCT = 0.25;
    public static double SLOW_MODE_TURN_PCT = 0.25;
    public static double SPEED = 1f;
    public static double TURN = 1f;

    // Slide

    // Arm
    public static double PICKUP_ARM_MIN = 0.185;
    public static double PICKUP_ARM_MAX = 0.755;
    public static double CLAW_MIN = 0.92;
    public static double CLAW_MAX = 0.6;
    public static double CLAW_ARM_DELTA = 0.03;

    // Gantry
    public static double GANTRY_ARM_MIN = 0.435;
    public static double GANTRY_ARM_MAX = 0.94;
    public static int GANTRY_LIFT_DELTA = 50;
    public static double GANTRY_ARM_KP = 0.06;
    public static double X_KP = 0.1;
    public static double X_MAX = 0.84;
    public static double X_MIN = 0.16;
    public static double X_CENTER = 0.54;
    public static double GANTRY_ARM_DELTA_MAX = 0.013;
    public static int SLIDE_UP = 100;


    // Robot Lift
    public static double LIFT_ROTATION_UP = 0.4;
    public static double LIFT_ROTATION_DOWN = 0;
    public static int LIFT_EXTEND_MAX = 13100;
    public static double LIFT_RETRACT_PCT = 0.4;
    public static double LIFT_ARM_KP = 0.1;
    public static double LIFT_POWER = 1f;

    public static double CLAW_KP = 0.3;
}
