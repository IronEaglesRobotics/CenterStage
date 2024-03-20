package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.TAG_CAMERA;
import static org.firstinspires.ftc.teamcode.util.Constants.TARGETING_WEBCAM;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.tearabite.ielib.common.Alliance;
import com.tearabite.ielib.localization.AprilTagPoseEstimator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

public class Camera {
    private OpenCvCamera targetingCamera;
    private PropDetectionPipeline prop;
    private boolean targetingCameraInitialized;
    private AprilTagProcessor aprilTag;
    private VisionPortal aprilTagPortal;
    private VisionPortal propPortal;
    private boolean initialized;
    private static final Pose2d TAG_CAMERA_OFFSETS = new Pose2d(-8.27, -0.95, 0);
    private AprilTagPoseEstimator aprilTagPoseEstimator;
    public static float PROP_REJECTION_VERTICAL_UPPER = 40;
    public static float PROP_REJECTION_VERTICAL_LOWER = 440;


    // Constructor
    public Camera(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.aprilTagPoseEstimator = AprilTagPoseEstimator.builder()
                .robotOffset(TAG_CAMERA_OFFSETS)
                .build();
        this.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        int[] viewportIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        VisionPortal.Builder aprilTagVisionPortalBuilder = new VisionPortal.Builder();
        aprilTagVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, TAG_CAMERA));
        aprilTagVisionPortalBuilder.setLiveViewContainerId(viewportIds[0]);
        aprilTagVisionPortalBuilder.setAutoStopLiveView(true);
        aprilTagVisionPortalBuilder.addProcessor(aprilTag);
        this.aprilTagPortal = aprilTagVisionPortalBuilder.build();

        this.prop = new PropDetectionPipeline();
        VisionPortal.Builder propVisionPortalBuilder = new VisionPortal.Builder();
        propVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, TARGETING_WEBCAM));
        propVisionPortalBuilder.setLiveViewContainerId(viewportIds[1]);
        propVisionPortalBuilder.setAutoStopLiveView(true);
        propVisionPortalBuilder.addProcessor(prop);
        this.propPortal = propVisionPortalBuilder.build();

        this.propPortal.resumeLiveView();
        this.aprilTagPortal.resumeLiveView();
        this.initialized = true;
    }

    public StarterPosition getStartingPosition() {
        Detection red = this.prop.getRed();
        Detection blue = this.prop.getBlue();

        double redArea = red.getArea();
        double blueArea = blue.getArea();

        return getStartingPosition(redArea > blueArea ? red : blue);
    }

    private static StarterPosition getStartingPosition(Detection detection) {
        if (detection == null) {
            return StarterPosition.UNKNOWN;
        }

        Point center = detection.getCenter();
        if (center == null) {
            return StarterPosition.UNKNOWN;
        }

        double centerX = detection.getCenter().x + 50;
        if (centerX < 33) {
            return StarterPosition.LEFT;
        } else if (centerX < 66) {
            return StarterPosition.CENTER;
        } else if (centerX < 100) {
            return StarterPosition.RIGHT;
        }

        return StarterPosition.UNKNOWN;
    }


    public enum StarterPosition {
        UNKNOWN, LEFT, CENTER, RIGHT
    }

    public AprilTagDetection getBestAprilTagDetection() {
        return this.aprilTag.getDetections()
                .stream().max((d1, d2) -> Float.compare(d2.decisionMargin, d1.decisionMargin))
                .orElse(null);
    }

    public Pose2d estimatePoseFromAprilTag() {
        AprilTagDetection detection = this.getBestAprilTagDetection();
        if (detection == null) {
            return null;
        }
        return this.aprilTagPoseEstimator.estimatePose(detection);
    }
}