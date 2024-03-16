package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Configurables {

    // CV Color Threshold Constants
    public static Color FTC_RED_LOWER = new Color(165, 80, 80);
    public static Color FTC_RED_UPPER = new Color(15, 255, 255);
    public static Color FTC_BLUE_LOWER = new Color(75, 40, 80);
    public static Color FTC_BLUE_UPPER = new Color(120, 255, 255);
    public static Color FTC_WHITE_LOWER = new Color(0, 0, 40);
    public static Color FTC_WHITE_UPPER = new Color(180, 30, 255);

    //Drive Speed
    @Config
    public static class driveSpeed {
        public static double SPEED = 1;
        public static double SLOWMODE_SPEED = 0.3;
        public static double TURN = .75;
        public static double SLOWMODE_TURN = 0.3;
    }

    //Camera Stuff
    @Config
    public static class camerStuff {
        public static double CAMERA_OFFSET_X = 0f;
        public static double DETECTION_AREA_MIN = 0.02f;
        public static double DETECTION_AREA_MAX = 0.3f;
        public static double DETECTION_LEFT_X = 0;
        public static double DETECTION_CENTER_X = .5;
        public static double DETECTION_RIGHT_X = 1;
        public static double SCORING_DISTANCE_FROM_APRIL_TAG = 6f;
    }

    //Auto Temp
    @Config
    public static class AuToDeV {
        //Things
        public static double X1 = 0, Y1 = 0, R1 = 0;
        public static double X2 = 0, Y2 = 0, R2 = 0;
        public static double X3 = 0, Y3 = 0, R3 = 0;
        public static double X4 = 0, Y4 = 0, R4 = 0;
        public static double X5 = 0, Y5 = 0, R5 = 0;

        //Pose2d
        public static Pose2d TEMP1 = new Pose2d(X1, Y1, Math.toRadians(R1));
        public static Pose2d TEMP2 = new Pose2d(X2, Y2, Math.toRadians(R2));
        public static Pose2d TEMP3 = new Pose2d(X3, Y3, Math.toRadians(R3));
        public static Pose2d TEMP4 = new Pose2d(X4, Y4, Math.toRadians(R4));
        public static Pose2d TEMP5 = new Pose2d(X5, Y5, Math.toRadians(R5));
    }
}