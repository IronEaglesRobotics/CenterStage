package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CAMERA_FORWARD_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CAMERA_ROTATION_DEG;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CAMERA_SIDE_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.WEBCAM_MINI_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

import lombok.NonNull;

@Config
public class Camera {
    public static float PROP_REJECTION_VERTICAL_UPPER = 480f * 0.33f;
    public static float PROP_REJECTION_VERTICAL_LOWER = 440;
    public static float PROP_REJECTION_HORIZONTAL_LEFT = 10;
    public static float PROP_REJECTION_HORIZONTAL_RIGHT = 630;
    private PropDetectionPipeline prop;
    private AprilTagProcessor aprilTag;
    private VisionPortal aprilTagPortal;
    private VisionPortal propPortal;
    private Telemetry telemetry;
    public static final Vector2d tag2Pose = new Vector2d(60, 36);
    public static final Vector2d tag5Pose = new Vector2d(60, -36);
    private boolean initialized;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        int[] viewportIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        VisionPortal.Builder aprilTagVisionPortalBuilder = new VisionPortal.Builder();
        aprilTagVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        aprilTagVisionPortalBuilder.setLiveViewContainerId(viewportIds[0]);
        aprilTagVisionPortalBuilder.setAutoStopLiveView(true);
        aprilTagVisionPortalBuilder.addProcessor(aprilTag);
        this.aprilTagPortal = aprilTagVisionPortalBuilder.build();

        this.prop = new PropDetectionPipeline();
        VisionPortal.Builder propVisionPortalBuilder = new VisionPortal.Builder();
        propVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_MINI_NAME));
        propVisionPortalBuilder.setLiveViewContainerId(viewportIds[1]);
        propVisionPortalBuilder.setAutoStopLiveView(true);
        propVisionPortalBuilder.addProcessor(prop);
        this.propPortal = propVisionPortalBuilder.build();

        this.propPortal.resumeLiveView();
        this.aprilTagPortal.resumeLiveView();
        this.initialized = true;
    }

    public Detection getProp() {
        telemetry.addData("Getting Prop - Alliance", this.getAlliance());
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

    public Pose2d estimatePoseFromAprilTag() {
        List<AprilTagDetection> aprilTagDetections = aprilTag.getDetections();
        if (aprilTagDetections == null) {
            return null;
        }

        // Currently `estimatePoseFromAprilTag` does not work for tags 7-10. Filter them out.
        // Also, filter out detections that don't have a field position.
        List<AprilTagDetection> filteredDetections = aprilTagDetections.stream()
                .filter(x -> x.id <= 6 && x.metadata != null && x.metadata.fieldPosition != null)
                .collect(Collectors.toList());

        if (filteredDetections.isEmpty()) {
            return null;
        }

        return estimatePoseFromAprilTag(filteredDetections.stream()
                .max((a,  b) -> (int)(a.decisionMargin - b.decisionMargin))
                .get());
    }

    private Pose2d estimatePoseFromAprilTag(@NonNull AprilTagDetection aprilTagDetection) {
        VectorF reference = aprilTagDetection.metadata.fieldPosition;
        AprilTagPoseFtc ftcPose = aprilTagDetection.ftcPose;

        double ax = reference.get(0);
        double ay = reference.get(1);
        double t = -Math.toRadians(ftcPose.yaw);
        double r = -ftcPose.range;
        double b = Math.toRadians(ftcPose.bearing);

        double x1 = r * cos(b);
        double y1 = r * sin(b);

        double x2 = x1 * cos(t) - y1 * sin(t);
        double y2 = x1 * sin(t) + y1 * cos(t);

        double cx = ax + x2;
        double cy = ay + y2;

        double t1 = t + Math.toRadians(CAMERA_ROTATION_DEG);
        double h = tan(t1);

        double rx = -CAMERA_FORWARD_OFFSET_IN * cos(t1) + CAMERA_SIDE_OFFSET_IN * sin(t1);
        double ry = -CAMERA_FORWARD_OFFSET_IN * sin(t1) - CAMERA_SIDE_OFFSET_IN * cos(t1);

        rx += cx;
        ry += cy;

//        AprilTagDetection foo = aprilTagDetection;
//        telemetry.addData("id", foo.id);
//        telemetry.addData("decisionMargin", foo.decisionMargin);
//        telemetry.addData("ax", foo.metadata.fieldPosition.get(0));
//        telemetry.addData("ay", foo.metadata.fieldPosition.get(1));
//        telemetry.addData("yaw", foo.ftcPose.yaw);
//        telemetry.addData("range", foo.ftcPose.range);
//        telemetry.addData("bearing", foo.ftcPose.bearing);
//        telemetry.addData("x1", x1);
//        telemetry.addData("y1", y1);
//        telemetry.addData("x2", x2);
//        telemetry.addData("y2", y2);
//        telemetry.addData("cx", cx);
//        telemetry.addData("cy", cy);
//        telemetry.addData("t1", t1);
//        telemetry.addData("h", h);
//        telemetry.addData("rx", rx);
//        telemetry.addData("ry", ry);
//        telemetry.update();

        return new Pose2d(rx, ry, h);
    }
}