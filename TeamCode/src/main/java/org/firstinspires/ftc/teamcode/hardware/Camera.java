package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.FORWARD_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SIDE_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;

import android.hardware.GeomagneticField;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

@Config
public class Camera {
    public static float PROP_REJECTION_VERTICAL_UPPER = 150;
    public static float PROP_REJECTION_VERTICAL_LOWER = 275;
    private PropDetectionPipeline prop;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
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

    public AprilTagDetection getAprilTag(int ... ids) {
        return this.aprilTag.getDetections()
                .stream()
                .filter(x -> Arrays.stream(ids).filter(id -> x.id == id).count() > 0)
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

    public Vector2d getPoseFromAprilTag(int ... ids) {
        if (ids == null || ids.length == 0) {
            ids = new int[]{2, 5};
        }

        AprilTagDetection aprilTagDetection = getAprilTag(ids);

        if (aprilTagDetection == null) {
            return null;
        }

        AprilTagPoseFtc ftcPose = aprilTagDetection.ftcPose;

        double ourPoseX;
        double ourPoseY;
        switch (aprilTagDetection.id) {
            case 2:
                ourPoseX = tag2Pose.getX() - ftcPose.y - FORWARD_OFFSET_IN;
                ourPoseY = tag2Pose.getY() - ftcPose.x - SIDE_OFFSET_IN;
                break;
            case 5:
                ourPoseX = tag5Pose.getX() - ftcPose.y - FORWARD_OFFSET_IN;
                ourPoseY = tag5Pose.getY() - ftcPose.x - SIDE_OFFSET_IN;
                break;
            default:
                ourPoseX = 0;
                ourPoseY = 0;
                break;
        }

        return new Vector2d(ourPoseX, ourPoseY);
    }
}