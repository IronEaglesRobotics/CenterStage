package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_AREA_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_AREA_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_CENTER_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_LEFT_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_RIGHT_X;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_BLUE_RANGE;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_RED_RANGE_1;
import static org.firstinspires.ftc.teamcode.util.Colors.FTC_RED_RANGE_2;
import static org.firstinspires.ftc.teamcode.util.Colors.WHITE;
import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContour;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.util.ScalarRange;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import lombok.Getter;
import lombok.Setter;

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

        Imgproc.erode(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        return getLargestContour(contours);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        int detectionLeftXPx = (int)((DETECTION_LEFT_X / 100) * onscreenWidth);
        int detectionCenterXPx = (int)((DETECTION_CENTER_X / 100) * onscreenWidth);
        int detectionRightXPx = (int)((DETECTION_RIGHT_X / 100) * onscreenWidth);

        canvas.drawLine(detectionLeftXPx, 0, detectionLeftXPx, canvas.getHeight(), WHITE);
        canvas.drawLine(detectionCenterXPx, 0, detectionCenterXPx, canvas.getHeight(), WHITE);
        canvas.drawLine(detectionRightXPx, 0, detectionRightXPx, canvas.getHeight(), WHITE);

        if (red.isValid()) {
            canvas.drawCircle((float)red.getCenterPx().x, (float)red.getCenterPx().y, 10, WHITE);
        }

        if (blue.isValid()) {
            canvas.drawCircle((float)blue.getCenterPx().x, (float)blue.getCenterPx().y, 10, WHITE);
        }
    }
}