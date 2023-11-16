package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.UP;

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
    public KhangSlides slides;
    public Arm arm;
    public double macroStartTime = 0;
    public int macroState = 0;
    public int runningMacro = 0; // 0 = no macro | 1 = low macro | 2 = mid macro | 3 = high macro | 4 = pickup macro
    public int lastMacro = 0;

    private boolean camEnabled = true;

    //Name the objects
    public Robot(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        slides = new KhangSlides(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        intake = new Intake(hardwareMap, UP);
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

//    public void resetMacroNoDunk(int pos, double runTime) {
//        switch(macroState) {
//            case(0):
//                macroStartTime = runTime;
//                macroState++;
//                break;
//            case(1):
//                if (runTime > macroStartTime) {
//                    macroState ++;
//                }
//                break;
//            case(2):
//                macroStartTime = runTime;
//                slides.setTarget(pos);
//                macroState = 0;
//                runningMacro = 0;
//                lastMacro = 0;
//        }
//    }
//
//    public void resetMacro(int pos, double runTime) {
//        switch(macroState) {
//            case(0):
//                macroStartTime = runTime;
//                macroState++;
//                break;
//            case(1):
//                if (runTime > macroStartTime) {
//                    macroState ++;
//                }
//                break;
//            case(2):
//                macroStartTime = runTime;
//                slides.setTarget(pos);
//                macroState = 0;
//                runningMacro = 0;
//                lastMacro = 0;
//        }
//    }
//
//    public void resetMacroEnd(int pos, double runTime) {
//        switch(macroState) {
//            case(0):
//                macroStartTime = runTime;
//                macroState++;
//                break;
//            case(1):
//                if (runTime > macroStartTime) {
//                    macroState ++;
//                }
//                break;
//            case(2):
//                macroStartTime = runTime;
//                slides.setTarget(pos);
//                macroState = 0;
//                runningMacro = 0;
//                lastMacro = 0;
//        }
//    }

    public Arm getArm() {
        return this.arm;
    }
    //return drive
    public SampleMecanumDrive getDrive() {
        return this.drive;
    }
}