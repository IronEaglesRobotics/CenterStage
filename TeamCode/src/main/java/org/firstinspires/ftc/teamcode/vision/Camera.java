package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.TARGETING_WEBCAM;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_WIDTH;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Camera {
    private final HardwareMap hardwareMap;
    private OpenCvCamera targetingCamera;
    private ColorDetectionPipeline targetingPipeline;
    private boolean targetingCameraInitialized;

    // Constructor
    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // Initiate the Targeting Camera
    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, TARGETING_WEBCAM), targetingCameraMonitorViewId);
        this.targetingPipeline = new ColorDetectionPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                targetingCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
                targetingCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    // Close the Targeting Camera
    public void stopTargetingCamera() {
        if (targetingCameraInitialized) {
            targetingCamera.closeCameraDeviceAsync(() -> targetingCameraInitialized = false);
        }
    }

    // Get the Red Goal Detection
    public Detection getRed() {
        return targetingCameraInitialized ? targetingPipeline.getRed() : INVALID_DETECTION;
    }

    public StarterPosition getStartingPosition() {
        if (!targetingCameraInitialized) {
           return StarterPosition.UNKNOWN;
        }

        Detection detection = this.getRed();
        if (detection == null || !detection.isValid()) {
            return StarterPosition.UNKNOWN;
        }
        Point center = detection.getCenter();
        if (center == null) {
            return StarterPosition.UNKNOWN;
        }

        double centerX = this.getRed().getCenter().x + 50;
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

}