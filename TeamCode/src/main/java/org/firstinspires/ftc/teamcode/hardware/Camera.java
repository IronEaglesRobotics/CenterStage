package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.WEBCAM_NAME;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_WIDTH;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.TargetingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Camera {
    private final HardwareMap hardwareMap;
    private OpenCvCamera targetingCamera;
    private TargetingPipeline targetingPipeline;
    private boolean targetingCameraInitialized;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
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

    public void stopTargetingCamera() {
        if (targetingCameraInitialized) {
            targetingCamera.closeCameraDeviceAsync(() -> targetingCameraInitialized = false);
        }
    }

    public Detection getRed() {
        return (targetingCameraInitialized ? targetingPipeline.getRed() : INVALID_DETECTION);
    }
}