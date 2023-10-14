package org.firstinspires.ftc.teamcode.util;

public class Color {
    public double h;
    public double s;
    public double v;

    public Color(double h, double s, double v) {
        this.h = h;
        this.s = s;
        this.v = v;
    }

    public double[] get() {
        return new double[]{h, s, v};
    }

    public double getH() {
        return h;
    }

    public double getS() {
        return s;
    }

    public double getV() {
        return v;
    }

}