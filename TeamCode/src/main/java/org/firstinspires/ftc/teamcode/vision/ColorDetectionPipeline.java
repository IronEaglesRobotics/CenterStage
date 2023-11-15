package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.Configurables.FTC_RED_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.FTC_RED_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MAX_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MIN_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContour;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetectionPipeline implements VisionProcessor {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat whiteMask = new Mat();
    Scalar redLower1;
    Scalar redUpper1;
    Scalar redLower2;
    Scalar redUpper2;

    private Detection red;

    // Init
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        red = new Detection(new Size(width, height), CV_MIN_GOAL_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed(input);

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    // Update the Red Goal Detection
    private void updateRed(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        redLower1 = new Scalar(FTC_RED_LOWER.getH(), FTC_RED_LOWER.getS(), FTC_RED_LOWER.getV());
        redUpper1 = new Scalar(180, FTC_RED_UPPER.getS(), FTC_RED_UPPER.getV());
        redLower2 = new Scalar(0, FTC_RED_LOWER.getS(), FTC_RED_LOWER.getV());
        redUpper2 = new Scalar(FTC_RED_UPPER.getH(), FTC_RED_UPPER.getS(), FTC_RED_UPPER.getV());
        Core.inRange(hsv, redLower1, redUpper1, redMask1);
        Core.inRange(hsv, redLower2, redUpper2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Detection newDetection = new Detection(input.size(), CV_MIN_GOAL_AREA, CV_MAX_GOAL_AREA);
            newDetection.setContour(contours.get(i));
            newDetection.draw(input, RED);
        }

        red.setContour(getLargestContour(contours));
        red.fill(input, RED);
    }

    public Detection getRed() {
        return this.red;
    }
}