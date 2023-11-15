package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Scalar;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class ScalarRange {
    private Scalar upper;
    private Scalar lower;
}
