package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

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
}