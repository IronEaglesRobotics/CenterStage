package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLACK_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MAX_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MIN_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLACK;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContour;

import org.firstinspires.ftc.teamcode.util.Color;
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
    Mat blueMask = new Mat();
    Mat blackMask = new Mat();
    Scalar redGoalLower1;
    Scalar redGoalUpper1;
    Scalar redGoalLower2;
    Scalar redGoalUpper2;
    Scalar blueGoalLower1;
    Scalar blueGoalUpper1;
    Scalar blueGoalLower2;
    Scalar blueGoalUpper2;
    Scalar blackGoalLower1;
    Scalar blackGoalUpper1;
    Scalar blackGoalLower2;
    Scalar blackGoalUpper2;

    private Detection red;
    private Detection blue;
    private Detection black;

    // Init
    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), CV_MIN_GOAL_AREA);
        blue = new Detection(input.size(), CV_MIN_GOAL_AREA);
        black = new Detection(input.size(), CV_MIN_GOAL_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed(input);
        updateBlue(input);
        updateBlack(input);
        return input;
    }

    // Update the Red Goal Detection
    private void updateRed(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        redGoalLower1 = new Scalar(CAMERA_RED_GOAL_LOWER.getH(), CAMERA_RED_GOAL_LOWER.getS(), CAMERA_RED_GOAL_LOWER.getV());
        redGoalUpper1 = new Scalar(180, CAMERA_RED_GOAL_UPPER.getS(), CAMERA_RED_GOAL_UPPER.getV());
        redGoalLower2 = new Scalar(0, CAMERA_RED_GOAL_LOWER.getS(), CAMERA_RED_GOAL_LOWER.getV());
        redGoalUpper2 = new Scalar(CAMERA_RED_GOAL_UPPER.getH(), CAMERA_RED_GOAL_UPPER.getS(), CAMERA_RED_GOAL_UPPER.getV());
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

    private void updateBlue(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        blueGoalLower1 = new Scalar(CAMERA_BLUE_GOAL_LOWER.getH(), CAMERA_BLUE_GOAL_LOWER.getS(), CAMERA_BLUE_GOAL_LOWER.getV());
        blueGoalUpper1 = new Scalar(180, CAMERA_BLUE_GOAL_UPPER.getS(), CAMERA_BLUE_GOAL_UPPER.getV());
        blueGoalLower2 = new Scalar(0, CAMERA_BLUE_GOAL_LOWER.getS(), CAMERA_BLUE_GOAL_LOWER.getV());
        blueGoalUpper2 = new Scalar(CAMERA_BLUE_GOAL_UPPER.getH(), CAMERA_BLUE_GOAL_UPPER.getS(), CAMERA_BLUE_GOAL_UPPER.getV());
        Core.inRange(hsv, blueGoalLower1, blueGoalUpper1, blueMask);
        Core.inRange(hsv, blueGoalLower2, blueGoalUpper2, blueMask);
        Imgproc.erode(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Detection newDetection = new Detection(input.size(), CV_MIN_GOAL_AREA, CV_MAX_GOAL_AREA);
            newDetection.setContour(contours.get(i));
            newDetection.draw(input, BLUE);
        }

        blue.setContour(getLargestContour(contours));
        blue.fill(input, BLUE);
    }

    private void updateBlack(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        blackGoalLower1 = new Scalar(CAMERA_BLACK_GOAL_LOWER.getH(), CAMERA_BLACK_GOAL_LOWER.getS(), CAMERA_BLACK_GOAL_LOWER.getV());
        blackGoalUpper1 = new Scalar(180, CAMERA_BLACK_GOAL_LOWER.getS(), CAMERA_BLACK_GOAL_LOWER.getV());
        blackGoalLower2 = new Scalar(0, CAMERA_BLACK_GOAL_LOWER.getS(), CAMERA_BLACK_GOAL_LOWER.getV());
        blackGoalUpper2 = new Scalar(CAMERA_BLACK_GOAL_LOWER.getH(), CAMERA_BLACK_GOAL_LOWER.getS(), CAMERA_BLACK_GOAL_LOWER.getV());
        Core.inRange(hsv, blackGoalLower1, blackGoalUpper1, blackMask);
        Core.inRange(hsv, blackGoalLower2, blackGoalUpper2, blackMask);
        Imgproc.erode(blackMask, blackMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blackMask, blackMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blackMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Detection newDetection = new Detection(input.size(), CV_MIN_GOAL_AREA, CV_MAX_GOAL_AREA);
            newDetection.setContour(contours.get(i));
            newDetection.draw(input, BLACK);
        }

        black.setContour(getLargestContour(contours));
        black.fill(input, BLACK);
    }

    public Detection getRed() {
        return this.red;
    }
    public Detection getBlue() {
        return this.blue;
    }
}