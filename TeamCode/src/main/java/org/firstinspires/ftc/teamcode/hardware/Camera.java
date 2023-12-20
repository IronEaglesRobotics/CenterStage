package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.TARGETING_WEBCAM;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_WIDTH;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.CameraPosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.TargetingPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;

// Class for the camera
public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera targetingCamera;
    private TargetingPipeline targetingPipeline;
    private boolean targetingCameraInitialized;

    private float decimation;
    private boolean needToSetDecimation;
    int numFramesWithoutDetection = 0;
    private boolean signalWebcamInitialized;
    private OpenCvCamera signalWebcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    ArrayList<AprilTagDetection> detections;
    static final double FEET_PER_METER = 3.28084;
    public CameraPosition cameraPosition;
    private final Object decimationSync = new Object();
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    // Constructor
    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // Initiate the Targeting Camera
    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, TARGETING_WEBCAM), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                targetingCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
                targetingCameraInitialized = true;
                FtcDashboard.getInstance().startCameraStream(signalWebcam, 0);
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

    // detect colors
    public Detection getRed() {
        return (targetingCameraInitialized ? targetingPipeline.getRed() : INVALID_DETECTION);
    }

    public Detection getBlue() {
        return (targetingCameraInitialized ? targetingPipeline.getBlue() : INVALID_DETECTION);
    }


    //return frame rate
    public int getFrameCount() {
        if (targetingCameraInitialized) {
            return targetingCamera.getFrameCount();
        } else {
            return 0;
        }
    }

    public int getMarkerId() {
        detections = aprilTagDetectionPipeline.getLatestDetections();

        // If there's been a new frame...
        if (detections != null) {
            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                return detections.get(0).id;

            }
        }
        return -1;
    }
}