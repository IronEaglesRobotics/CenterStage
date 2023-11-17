package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_CENTER_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_LEFT_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.DETECTION_RIGHT_X;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import lombok.Getter;
import lombok.Setter;

public class Camera {
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

    public CenterStageCommon.PropLocation getPropLocation() {
        Detection prop = this.getProp();
        if (prop == null || !prop.isValid()) {
            return CenterStageCommon.PropLocation.Unknown;
        }

        double x = prop.getCenter().x + 50;

        if (x <= DETECTION_LEFT_X) {
            return CenterStageCommon.PropLocation.Left;
        }
        if (x <= DETECTION_CENTER_X) {
            return CenterStageCommon.PropLocation.Center;
        }
        if (x <= DETECTION_RIGHT_X) {
            return CenterStageCommon.PropLocation.Right;
        }

        return CenterStageCommon.PropLocation.Unknown;
    }

    public AprilTagDetection getAprilTag(int id) {
        return this.aprilTag.getDetections()
                .stream()
                .filter(x -> x.id == id)
                .findFirst()
                .orElse(null);
    }

    public void setAlliance(CenterStageCommon.Alliance alliance) {
        this.prop.setAlliance(alliance);
    }

    public CenterStageCommon.Alliance getAlliance() {
        return this.prop != null ? this.prop.getAlliance() : null;
    }
}