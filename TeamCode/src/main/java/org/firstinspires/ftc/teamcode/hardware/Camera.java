package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.tearabite.ielib.localization.AprilTagPoseEstimator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Camera {
    private static final String REAR_WEBCAM_NAME = "rearWebcam";
    public static final String FRONT_WEBCAM_NAME = "frontWebcam";
    private static final Pose2d REAR_CAMERA_OFFSETS = new Pose2d(9.75, 0, Math.PI);
    public static float PROP_REJECTION_VERTICAL_UPPER = 480f * 0.33f;
    public static float PROP_REJECTION_VERTICAL_LOWER = 440;
    private PropDetectionPipeline prop;
    private AprilTagProcessor aprilTag;
    private VisionPortal aprilTagPortal;
    private VisionPortal propPortal;
    private boolean initialized;
    private AprilTagPoseEstimator aprilTagPoseEstimator;

    public Camera(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.aprilTagPoseEstimator = AprilTagPoseEstimator.builder()
                .robotOffset(REAR_CAMERA_OFFSETS)
                .build();
        this.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        int[] viewportIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        VisionPortal.Builder aprilTagVisionPortalBuilder = new VisionPortal.Builder();
        aprilTagVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, REAR_WEBCAM_NAME));
        aprilTagVisionPortalBuilder.setLiveViewContainerId(viewportIds[0]);
        aprilTagVisionPortalBuilder.setAutoStopLiveView(true);
        aprilTagVisionPortalBuilder.addProcessor(aprilTag);
        this.aprilTagPortal = aprilTagVisionPortalBuilder.build();

        this.prop = new PropDetectionPipeline();
        VisionPortal.Builder propVisionPortalBuilder = new VisionPortal.Builder();
        propVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, FRONT_WEBCAM_NAME));
        propVisionPortalBuilder.setLiveViewContainerId(viewportIds[1]);
        propVisionPortalBuilder.setAutoStopLiveView(true);
        propVisionPortalBuilder.addProcessor(prop);
        this.propPortal = propVisionPortalBuilder.build();

        this.propPortal.resumeLiveView();
        this.aprilTagPortal.resumeLiveView();
        this.initialized = true;
    }

    public Detection getProp() {
        if (!initialized || getAlliance() == null) {
            return INVALID_DETECTION;
        }

        switch (getAlliance()) {
            case Blue:
                Detection blue = this.prop.getBlue();
                return blue != null && blue.isValid() ? blue : INVALID_DETECTION;
            case Red:
                Detection red = this.prop.getRed();
                return red != null && red.isValid() ? red : INVALID_DETECTION;
        }

        return INVALID_DETECTION;
    }

    public void setAlliance(CenterStageCommon.Alliance alliance) {
        this.prop.setAlliance(alliance);
    }

    public CenterStageCommon.Alliance getAlliance() {
        return this.prop != null ? this.prop.getAlliance() : null;
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