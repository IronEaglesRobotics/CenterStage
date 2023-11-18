package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

@Config
public class Camera {
    public static float PROP_REJECTION_VERTICAL_UPPER = 175;
    public static float PROP_REJECTION_VERTICAL_LOWER = 300;
    private PropDetectionPipeline prop;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

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
        this.prop = new PropDetectionPipeline();
        this.visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), aprilTag, prop);
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

    public AprilTagDetection getAprilTag(int id) {
        return this.aprilTag.getDetections()
                .stream()
                .filter(x -> x.id == id)
                .findFirst()
                .orElse(null);
    }

    public double getDistanceToAprilTag(int id, double rejectAbove, double rejectBelow) {
        for (int i = 0; i < 10; i++) {
            AprilTagDetection aprilTagDetection = getAprilTag(id);
            if (aprilTagDetection != null) {
                if (aprilTagDetection.ftcPose.y < rejectAbove
                        && aprilTagDetection.ftcPose.y > rejectBelow) {
                    return aprilTagDetection.ftcPose.y;
                }
            }
        }
        return Double.MAX_VALUE;
    }

    public void setAlliance(CenterStageCommon.Alliance alliance) {
        this.prop.setAlliance(alliance);
    }

    public CenterStageCommon.Alliance getAlliance() {
        return this.prop != null ? this.prop.getAlliance() : null;
    }
}