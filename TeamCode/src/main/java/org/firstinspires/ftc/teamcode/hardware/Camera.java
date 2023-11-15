package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.ColorDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Camera {
    private ColorDetectionPipeline colorDetectionPipeline;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    private boolean initialized;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        this.aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        this.colorDetectionPipeline = new ColorDetectionPipeline();
        this.visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), aprilTag, colorDetectionPipeline);
        this.initialized = true;
    }

    public Detection getRed() {
        return (initialized ? colorDetectionPipeline.getRed() : INVALID_DETECTION);
    }
}