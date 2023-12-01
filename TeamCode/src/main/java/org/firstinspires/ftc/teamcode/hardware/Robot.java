package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.UP;
import static org.firstinspires.ftc.teamcode.hardware.DoorPos.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.DoorPos.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.robby.Slides;
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
    public double macroStartTime = 0;
    public int macroState = 0;
    public int runningMacro = 0; // 0 = no macro | 1 = tier 1 | 2 = tier 2 | 3 = tier 3 | 4 = return
    public int lastMacro = 0;

    private boolean camEnabled = true;

    //Name the objects
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        intake = new Intake(hardwareMap, UP);
        slides = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);
        camEnabled = true;
    }

    public void extendMacro(Slides.Position slidePos, double runTime) {
        switch(macroState) {
            case(0):
                macroStartTime = runTime;
                slides.setTarget(slidePos);
                arm.setDoor(CLOSE);
                macroState ++;
                break;
            case(1):
                if (runTime > macroStartTime + 1) {
                    macroState ++;
                }
                break;
            case(2):
                macroStartTime = runTime;
                arm.setArmPos(Arm.Position.SCORE);
                arm.setWristPos(Arm.Position.SCORE);
                macroState = 0;
                lastMacro = runningMacro;
                runningMacro = 0;
                break;
        }
    }

    public void resetMacro(Slides.Position pos, double runTime) {
        switch(macroState) {
            case(0):
                macroStartTime = runTime;
                arm.setDoor(OPEN);
                macroState++;
                break;
                //Ind_sleeper
            case(1):
                if (runTime > macroStartTime + 1) {
                    macroState ++;
                }
                break;
            case(2):
                macroStartTime = runTime;
                arm.setArmPos(Arm.Position.INTAKE);
                arm.setWristPos(Arm.Position.INTAKE);
                macroState++;
                break;
            case(3):
                if (runTime > macroStartTime + 1.1) {
                    macroState ++;
                }
                break;
            case(4):
                macroStartTime = runTime;
                slides.setTarget(pos);
                macroState = 0;
                lastMacro = runningMacro;
                runningMacro = 0;
        }
    }

    public void update(double runTime) {
        drive.update();
        slides.update(runTime);
        arm.update();
    }

    public String getTelemetry() {
        return "Telemetry?";
    }
}