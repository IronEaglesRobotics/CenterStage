package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Scalar;

public class ScalarRange {
    private Scalar upper;
    private Scalar lower;

    public ScalarRange(Scalar upper, Scalar lower) {
        this.upper = upper;
        this.lower = lower;
    }

    public Scalar getUpper() {
        return this.upper;
    }

    public Scalar getLower() {
        return this.lower;
    }
}