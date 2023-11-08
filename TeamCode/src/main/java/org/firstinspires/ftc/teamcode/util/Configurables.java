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

    //Servo Positions
    public static double UNLOCKSPEED = .1;
    public static double LOCKSPEED = .25;
    public static double UNLOCK = .6;
    public static double LOCK = 0.4;
    public static double ARMREST = .88;
    public static double ARMSCORE = 0.15;
    public static double ARMACCSCORE = 0.02;
    public static double PICKUP = .935;
    public static double WRISTPICKUP = 0.28;
    public static double WRISTSCORE = .96;
    public static double OPEN = 0.53;
    public static double BIGOPEN = 0.45;
    public static double CLOSE = 0.6;

    //Drive Speed
    public static double SPEED = 1;
    public static double SLOWMODE_SPEED = 0.5;
    public static double TURN = 1;
    public static double SLOWMODE_TURN = 0.75;

}