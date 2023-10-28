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

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TargetingPipeline extends OpenCvPipeline {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat whiteMask = new Mat();
    Scalar redGoalLower1;
    Scalar redGoalUpper1;
    Scalar redGoalLower2;
    Scalar redGoalUpper2;

    private Detection red;

    // Init
    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), CV_MIN_GOAL_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed(input);

        return input;
    }

    // Update the Red Goal Detection
    private void updateRed(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        redGoalLower1 = new Scalar(FTC_RED_LOWER.getH(), FTC_RED_LOWER.getS(), FTC_RED_LOWER.getV());
        redGoalUpper1 = new Scalar(180, FTC_RED_UPPER.getS(), FTC_RED_UPPER.getV());
        redGoalLower2 = new Scalar(0, FTC_RED_LOWER.getS(), FTC_RED_LOWER.getV());
        redGoalUpper2 = new Scalar(FTC_RED_UPPER.getH(), FTC_RED_UPPER.getS(), FTC_RED_UPPER.getV());
        Core.inRange(hsv, redGoalLower1, redGoalUpper1, redMask1);
        Core.inRange(hsv, redGoalLower2, redGoalUpper2, redMask2);
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