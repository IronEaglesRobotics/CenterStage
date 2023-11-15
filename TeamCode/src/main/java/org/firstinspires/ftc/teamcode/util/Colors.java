package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Scalar;
import android.graphics.Color;
public class Colors {
    // CV Color Threshold Constants
    public static Scalar FTC_RED_LOWER = new Scalar(165, 80, 80);
    public static Scalar FTC_RED_UPPER = new Scalar(15, 255, 255);
    public static ScalarRange FTC_RED_RANGE_1 = new ScalarRange(FTC_RED_UPPER, FTC_RED_LOWER);
    public static ScalarRange FTC_RED_RANGE_2 = new ScalarRange(FTC_RED_UPPER, FTC_RED_LOWER);
    public static Scalar FTC_BLUE_LOWER = new Scalar(75, 40, 80);
    public static Scalar FTC_BLUE_UPPER = new Scalar(120, 255, 255);
    public static ScalarRange FTC_BLUE_RANGE = new ScalarRange(FTC_BLUE_UPPER, FTC_RED_LOWER);
    public static Scalar FTC_WHITE_LOWER = new Scalar(0, 0, 40);
    public static Scalar FTC_WHITE_UPPER = new Scalar(180, 30, 255);

    public static OpenCVUtil.LinePaint RED = new OpenCVUtil.LinePaint(Color.RED);

    public static OpenCVUtil.LinePaint BLUE = new OpenCVUtil.LinePaint(Color.BLUE);
    public static OpenCVUtil.LinePaint BLACK = new OpenCVUtil.LinePaint(Color.BLACK);
    public static OpenCVUtil.LinePaint WHITE = new OpenCVUtil.LinePaint(Color.WHITE);
}