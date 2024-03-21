package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

@Config
public class LEDs {
    public static int NUMBER = 1;//80 solid
    public static int RED = 80;
    public static int REDFULL = 3;
    public static int REDSCORING = 5;//5
    public static int REDINIT = 79;
    public static int BLUE = 93;
    public static int BLUEFULL = 2;
    public static int BLUESCORING = 5;//5
    public static int BLUEINIT = 99;

    public static int yellow = 81;
    public static int lime = 87;
    public static int white = 97;
    public static int purple = 78;

    //yellow:81, green: 87, white:97 , purple:78 ,
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    public LEDs(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LEDs");
        setPattern();
    }

    public void setPattern() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(NUMBER);
        blinkinLedDriver.setPattern(pattern);
    }

    public void setPattern(int patternNumber) {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternNumber);
        blinkinLedDriver.setPattern(pattern);
    }

    public void nextPattern() {
        pattern = pattern.next();
        blinkinLedDriver.setPattern(pattern);
    }

    public void previousPattern() {
        pattern = pattern.previous();
        blinkinLedDriver.setPattern(pattern);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Pattern: %s", pattern.toString());
    }
}