package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static final String LIFT_MOTOR_NAME = "lift";
    public static final String CLAW_ARM_LEFT_NAME = "clawArmLeft";
    public static final String CLAW_ARM_RIGHT_NAME = "clawArmRight";
    public static final String CLAW_NAME = "claw";
    public static final String GANTRY_X_NAME = "gantryX";
    public static final String GANTRY_ARM_NAME = "gantryArm";
    public static final String GANTRY_SCREW_NAME = "screw";

    // Lift
    public static int LIFT_UP = 100;

    // Arm
    public static double PICKUP_ARM_MIN = 0;
    public static double PICKUP_ARM_MAX = 1;
    public static double CLAW_MIN = 0;
    public static double CLAW_MAX = 1;

    // Gantry
    public static double GANTRY_ARM_MIN = 0;
    public static double GANTRY_ARM_MAX = 0;
    public static double[] GANTRY_SCREW_POSITIONS = new double[] { 0, 0.1, 0.2, 1.0 };
}
