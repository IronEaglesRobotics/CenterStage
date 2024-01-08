package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

public class Configurables {
    // Robot Constants
    public static double R_ARM_POWER = 0.2;
    public static double R_ARM_SPEED = 20;
    public static int R_ARM_DEFAULT_POS = 0;
    public static int R_ARM_UP_POS = 221;
    public static int R_ARM_ALMOST_DOWN_POS = 650;
    public static int R_ARM_DOWN_POS = 750;
    public static double R_CLAW_CLOSED = 0.13;
    public static double R_CLAW_OPEN = 0.7;
    public static double R_INTAKE_SPEED = 0.9;
    public static double R_INTAKE_SHIELD_UP = 0.17;//0.05
    public static double R_INTAKE_SHIELD_DOWN = 0.68;//0.95
    public static double R_INTAKE_SHIELD_SPEED = 0.04;
    public static double R_SHOOTER_GOAL_POWER = 0.66;
    public static double R_SHOOTER_MID_GOAL_POWER = 0.54;
    public static double R_SHOOTER_POWERSHOT_POWER = 0.57;
    public static double R_PUSHER_CLOSED = 0.35;
    public static double R_PUSHER_OPEN = 0.55;
    public static double R_PUSHER_DELAY = 0.15;

    // Auto Aim Constants
    public static double AUTO_AIM_OFFSET_X = 5;
    public static double AUTO_AIM_WAIT = 0.2;
    public static PIDFCoefficients AUTO_AIM_PID = new PIDFCoefficients(0.009, 0.3, 0.0019, 0);
    public static double AUTO_AIM_ACCEPTABLE_ERROR = 2;
    public static double AUTO_AIM_MIN_POWER = 0.14;

    // CV Color Threshold Constants
    public static Color CAMERA_RED_GOAL_LOWER       = new Color(165, 80, 80);
    public static Color CAMERA_RED_GOAL_UPPER       = new Color(15, 255, 255);
    public static Color CAMERA_RED_POWERSHOT_LOWER  = new Color(165, 80, 80);
    public static Color CAMERA_RED_POWERSHOT_UPPER  = new Color(15, 255, 255);
    public static Color CAMERA_BLUE_GOAL_LOWER      = new Color(75, 40, 80);
    public static Color CAMERA_BLUE_GOAL_UPPER      = new Color(120, 255, 255);
    public static Color CAMERA_BLACK_GOAL_LOWER      = new Color(0, 0, 0);
    public static Color CAMERA_BLACK_GOAL_UPPER      = new Color(360, 0, 0);
    public static Color CAMERA_BLUE_POWERSHOT_LOWER = new Color(75, 30, 50);
    public static Color CAMERA_BLUE_POWERSHOT_UPPER = new Color(120, 255, 255);
    public static Color CAMERA_ORANGE_LOWER         = new Color(0, 70, 50);
    public static Color CAMERA_ORANGE_UPPER         = new Color(60, 255, 255);
    public static Color CAMERA_WHITE_LOWER          = new Color(0, 0, 40);
    public static Color CAMERA_WHITE_UPPER          = new Color(180, 30, 255);

    // CV Detection Constants
    public static double CV_MIN_STARTERSTACK_AREA = 0;
    public static double CV_MIN_STARTERSTACK_SINGLE_AREA = 0.08;
    public static double CV_MIN_STARTERSTACK_QUAD_AREA = 1.3;
    public static double CV_MIN_GOAL_AREA = 0;
    public static double CV_MAX_GOAL_AREA = 0.3;
    public static double CV_MIN_POWERSHOT_AREA = 0.001;
    public static double CV_MAX_POWERSHOT_AREA = 0.05;
    public static Rect CV_STARTERSTACK_LOCATION = new Rect(75, 50, 190, 90);
    public static Point CV_POWERSHOT_OFFSET = new Point(-3, -20); // offset from the bottom left of the goal to the top right of the powershot box (for red)
    public static Size CV_POWERSHOT_DIMENSIONS = new Size(100, 50);

    public static Size CV_GOAL_PROPER_ASPECT = new Size(11, 8.5);
    public static double CV_GOAL_PROPER_AREA = 1.25;
    public static double CV_GOAL_ALLOWABLE_AREA_ERROR = 1;
    public static double CV_GOAL_ALLOWABLE_SOLIDARITY_ERROR = 1;
    public static double CV_GOAL_CUTOFF_Y_LINE = 65;
    public static double CV_GOAL_PROPER_HEIGHT = 107;
    public static double CV_GOAL_MIN_CONFIDENCE = 3;

    public static Color CV_POWERSHOT_OFFSETS_RED = new Color(-40, -30, -19);
    public static Color CV_POWERSHOT_OFFSETS_BLUE = new Color(40, 30, 19);

    // Old CV Detection Constants
    public static double CV_GOAL_SIDE_ALLOWABLE_Y_ERROR = 20;
    public static double CV_GOAL_SIDE_ALLOWABLE_SIZE_ERROR = 100;
    public static Size CV_GOAL_SIDE_ASPECT_RATIO = new Size(6.5,15.5);
    public static double CV_GOAL_SIDE_ALLOWABLE_ASPECT_ERROR = 10;

    public static double CAMERA_OFFSET_X = 0f;
    public static double DETECTION_AREA_MIN = 0.02f;
    public static double DETECTION_AREA_MAX = 0.3f;
    public static double DETECTION_LEFT_X = 0;
    public static double DETECTION_CENTER_X = .5;
    public static double DETECTION_RIGHT_X = 1;
    public static double SCORING_DISTANCE_FROM_APRIL_TAG = 6f;
}