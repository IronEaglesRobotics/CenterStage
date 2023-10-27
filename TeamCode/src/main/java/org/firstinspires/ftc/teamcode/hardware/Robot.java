package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class Robot {
    //set to public Drive to revert
    public SampleMecanumDrive drive;
    public Camera camera;
    public Intake intake;
    private boolean camEnabled = true;

    public Robot(HardwareMap hardwareMap) {
        //change this to new Drive to revert
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        intake = new Intake(hardwareMap);
        camEnabled = true;
        }

    public void update(double runTime) {
        drive.update();
    }

    public String getTelemetry() {
        Encoder slide = null;
        return String.format("position: %s", slide.getCurrentPosition());
    }

    //set to public Drive to revert
    public SampleMecanumDrive getDrive() {
        return this.drive;
    }
}