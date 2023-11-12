package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class Robot {

    //Create each object
    public SampleMecanumDrive drive;
    public Camera camera;
    public Intake intake;
    public Slides slides;
    public Arm arm;
    private boolean camEnabled = true;

    //Name the objects
    public Robot(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        slides = new Slides(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        intake = new Intake(hardwareMap);
        camEnabled = true;
        }

        //Update on runtime and dive
    public void update(double runTime) {
        drive.update();
    }

    //Return position to control hub
    public String getTelemetry() {
        Encoder slide = null;
        return String.format("position: %s", slide.getCurrentPosition());
    }

    public Arm getArm() {
        return this.arm;
    }
    //return drive
    public SampleMecanumDrive getDrive() {
        return this.drive;
    }
}