package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.hardware.Camera.PROP_REJECTION_HORIZONTAL_LEFT;
import static org.firstinspires.ftc.teamcode.hardware.Camera.PROP_REJECTION_HORIZONTAL_RIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Camera.PROP_REJECTION_VERTICAL_LOWER;
import static org.firstinspires.ftc.teamcode.hardware.Camera.PROP_REJECTION_VERTICAL_UPPER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_AREA_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_AREA_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_LEFT_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_RIGHT_X;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_BLUE_RANGE;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_RED_RANGE_1;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_RED_RANGE_2;
import static org.firstinspires.ftc.teamcode.util.Colors.RED;
import static org.firstinspires.ftc.teamcode.util.Colors.WHITE;
import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLACK;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContour;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Gantry;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.util.ScalarRange;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import lombok.Getter;
import lombok.Setter;
import opmodes.Test;

public class PropDetectionPipeline implements VisionProcessor {
    @Getter
    @Setter
    CenterStageCommon.Alliance alliance;
    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat tmpMask = new Mat();
    @Getter
    private Detection red;
    @Getter
    private Detection blue;

    // Init
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.red = new Detection(new Size(width, height), DETECTION_AREA_MIN, DETECTION_AREA_MAX);
        this.blue = new Detection(new Size(width, height), DETECTION_AREA_MIN, DETECTION_AREA_MAX);
    }

    // Process each frame that is received from the webcam
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        if (alliance == CenterStageCommon.Alliance.Red) {
            red.setContour(detect(FTC_RED_RANGE_1, FTC_RED_RANGE_2));
        }

        if (alliance == CenterStageCommon.Alliance.Blue) {
            blue.setContour(detect(FTC_BLUE_RANGE));
        }

        return input;
    }

    private Mat zeros;
    private Mat zeros(Size size, int type) {
        if (this.zeros == null) {
            this.zeros = Mat.zeros(size, type);
        }

        return this.zeros;
    }

    private MatOfPoint detect(ScalarRange... colorRanges) {
        mask.release();
        for (ScalarRange colorRange : colorRanges) {
            Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), tmpMask);
            if (mask.empty() || mask.rows() <= 0) {
                Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), mask);
            }
            Core.add(mask, tmpMask, mask);
        }

        Imgproc.rectangle(mask, new Point(0,0), new Point(mask.cols() - 1, (int)PROP_REJECTION_VERTICAL_UPPER), BLACK, -1);
        Imgproc.rectangle(mask, new Point(0,(int)PROP_REJECTION_VERTICAL_LOWER), new Point(mask.cols() - 1, mask.rows() -1), BLACK, -1);
        Imgproc.rectangle(mask, new Point(0,0), new Point(PROP_REJECTION_HORIZONTAL_LEFT, mask.rows() - 1), BLACK, -1);
        Imgproc.rectangle(mask, new Point(PROP_REJECTION_HORIZONTAL_RIGHT, 0), new Point(mask.cols() - 1, mask.rows() - 1), BLACK, -1);
        
        Imgproc.erode(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        return getLargestContour(contours);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawLine(0, PROP_REJECTION_VERTICAL_LOWER, canvas.getWidth(), PROP_REJECTION_VERTICAL_LOWER, WHITE);
        canvas.drawLine(0, PROP_REJECTION_VERTICAL_UPPER, canvas.getWidth(), PROP_REJECTION_VERTICAL_UPPER, WHITE);

        if (red != null && red.isValid()) {
            Point center = red.getCenterPx();
            if (center.y < PROP_REJECTION_VERTICAL_LOWER
                    && center.y > PROP_REJECTION_VERTICAL_UPPER) {
                canvas.drawCircle((float)red.getCenterPx().x, (float)red.getCenterPx().y, 10, WHITE);
            } else {
                canvas.drawCircle((float)red.getCenterPx().x, (float)red.getCenterPx().y, 10, RED);
            }
        }

        if (blue != null && blue.isValid()) {
            Point center = blue.getCenterPx();
            if (center.y < PROP_REJECTION_VERTICAL_LOWER
                    && center.y > PROP_REJECTION_VERTICAL_UPPER) {
                canvas.drawCircle((float)blue.getCenterPx().x, (float)blue.getCenterPx().y, 10, WHITE);
            } else {
                canvas.drawCircle((float)blue.getCenterPx().x, (float)blue.getCenterPx().y, 10, RED);
            }
        }
    }
}