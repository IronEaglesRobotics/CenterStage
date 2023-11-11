package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.vision.Detection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Constants {
    // CV Color Constants
    public static Scalar RED = new Scalar(255, 0, 0);
    public static Scalar GREEN = new Scalar(0, 255, 0);
    public static Scalar BLUE = new Scalar(0, 0, 255);
    public static Scalar WHITE = new Scalar(255, 255, 255);
    public static Scalar GRAY = new Scalar(80, 80, 80);
    public static Scalar BLACK = new Scalar(0, 0, 0);
    public static Scalar ORANGE = new Scalar(255, 165, 0);
    public static Scalar YELLOW = new Scalar(255, 255, 0);
    public static Scalar PURPLE = new Scalar(128, 0, 128);

    // CV Structuring Constants
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Size BLUR_SIZE = new Size(7, 7);

    // CV Camera Constants
    public static final int WEBCAM_WIDTH = 320;
    public static final int WEBCAM_HEIGHT = 240;
    public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    // CV Invalid Detection Constants
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;
    public static final Detection INVALID_DETECTION = new Detection(new Size(0, 0), 0);

    // Hardware Name Constants
    public static final String WHEEL_FRONT_LEFT = "frontLeft";
    public static final String WHEEL_FRONT_RIGHT = "frontRight";
    public static final String WHEEL_BACK_LEFT = "backLeft";
    public static final String WHEEL_BACK_RIGHT = "backRight";
    public static final String ARM = "wobbler";
    public static final String CLAW = "claw";
    public static final String INTAKE = "intake";
    public static final String INTAKE_SECONDARY = "secondary";
    public static final String INTAKE_SHIELD = "shield";
    public static final String SHOOTER = "wheel";
    public static final String PUSHER = "pusher";
    public static final String STACK_WEBCAM = "Stack Webcam";
    public static final String TARGETING_WEBCAM = "Targeting Webcam";
    public static final String IMU_SENSOR = "imu";
    public static final String LEFTHANG = "leftHang";
    public static final String RIGHTHANG = "rightHang";
    public static final String LEFTARM = "leftArm";
    public static final String RIGHTARM = "rightArm";
    public static final String WRIST = "wrist";
    public static final String PLANE = "plane";
}