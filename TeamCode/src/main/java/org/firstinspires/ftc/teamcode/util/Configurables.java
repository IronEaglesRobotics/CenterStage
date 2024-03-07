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
    public static double ARMREST = 0.91;
    public static double ARMSCORE = 0.4;
    public static double ARMACCSCORE = .57;
    public static double ARMPICKUPSTACK = 0.815;
    public static double PICKUP = 0.835;
    public static double WRISTPICKUP = 0.3;
    public static double WRISTSCORE = .98;
    public static double OPEN = 0.483;
    public static double BIGOPEN = 0.65;
    public static double CLOSE = .51;
    public static double PLANELOCK = 0.1;
    public static double PLANELAUNCH = 0.9;

    //Drive Speed
    public static double SPEED = 1;
    public static double SLOWMODE_SPEED = 0.3;
    public static double TURN = .75;
    public static double SLOWMODE_TURN = 0.3;

    //Motor Positions
    public static double SLIDE_POWER_UP = 1;
    public static double SLIDE_POWER_DOWN = .7;
    public static int SLIDELAYERONE = 60;
    public static int SLIDEAUTOSTACKS = 250;
    public static int SLIDEUP = 1150;
    public static int HANGRELEASE = 1550;
    public static int HANG = 0;
    public static int HANGPLANE = 1150;
    public static int SLIDELAYERTWO = 350;
    public static int SLIDESTACK = 80;
    public static int SLIDEPICKUPSTACKSTWO = 30;

}