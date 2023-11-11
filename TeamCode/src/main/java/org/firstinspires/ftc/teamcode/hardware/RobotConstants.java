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
    public static final String GANTRY_X_NAME = "gantryX";
    public static final String GANTRY_ARM_NAME = "gantryArm";
    public static final String GANTRY_SCREW_NAME = "screw";
    public static final String ROBOT_LIFT_ROTATION_NAME = "liftRotation";
    public static final String ROBOT_LIFT_LIFT_NAME = "liftLift";
    public static final String WEBCAM_NAME = "webcam";

    // Slide
    public static int SLIDE_UP = 100;

    // Arm
    public static double PICKUP_ARM_MIN = 0.17;
    public static double PICKUP_ARM_MAX = 0.75;
    public static double CLAW_MIN = 0.9;
    public static double CLAW_MAX = 0.6;
    public static double CLAW_ARM_DELTA = 0.008;

    // Gantry
    public static double GANTRY_ARM_MIN = 0.43;
    public static double GANTRY_ARM_MAX = 0.95;
    public static int GANTRY_LIFT_DELTA = 50;
    public static double GANTRY_X_DELTA = 0.01;
    public static double GANTRY_CENTER = 0.5;
    public static double GANTRY_ARM_KP = 0.03;
    public static double GANTRY_ARM_KI = 0f;
    public static double GANTRY_ARM_KD = 0f;
    public static double GANTRY_ARM_DELTA_MAX = 0.0025;

    // Robot Lift
    public static double LIFT_ROTATION_UP = 0.4;
    public static double LIFT_ROTATION_DOWN = 0;
    public static int LIFT_EXTEND_MAX = 13100;
    public static double LIFT_RETRACT_PCT = 0.1;
    public static double LIFT_ARM_KP = 0.038;
    public static double LIFT_POWER = 1f;

}
