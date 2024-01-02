package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.UP;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
public class Robot {

    //Create each object
    public SampleMecanumDrive drive;
    public Camera camera;
    public Intake intake;
    public Slides slides;
    public Arm arm;
    public endGame_Mechs endGameMechs;

    public double macroStartTime = 0;
    public int macroState = 0;
    public int runningMacro = 0; // 0 = no macro | 1 = tier 1 | 2 = tier 2 | 3 = tier 3 | 4 = return
    public int lastMacro = 0;

    private boolean camEnabled = true;
    public static double slideWait = 1.5;
    public static double scoreWait = 0.65;
    public static double armWait = 2.0;

    //Name the objects
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        intake = new Intake(hardwareMap, UP);
        slides = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);
        endGameMechs = new endGame_Mechs(hardwareMap);
        camEnabled = true;
    }

    public void extendMacro(int slidePos, double runTime) {
        switch(macroState) {
            case(0):
                macroStartTime = runTime;
                slides.setTarget(slidePos);
                arm.setDoor(CLOSE);
                macroState ++;
                break;
            case(1):
                if (runTime > macroStartTime + slideWait || slides.atTarget()) {
                    macroState ++;
                }
                break;
            case(2):
                arm.setArmPos(Arm.Position.SCORE);
                arm.setWristPos(Arm.Position.SCORE);
                macroState++;
                break;
            case (3):
                if(arm.armAtPosition()){
                    macroState = 0;
                    lastMacro = runningMacro;
                    runningMacro = 0;
                }
                break;
        }
    }

    public void resetMacro(int slidePos, double runTime) {
        switch(macroState) {
            case(0):
                macroStartTime = runTime;
                arm.setDoor(OPEN);
                macroState++;
                break;
            case(1):
                if (runTime > macroStartTime + scoreWait) {
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
                if (/*runTime > macroStartTime + armWait || */arm.armAtPosition()) {
                    macroState ++;
                }
                break;
            case(4):
                slides.setTarget(slidePos);
                macroState++;
                break;
            case (5):
                if (slides.atTarget()){
                    macroState = 0;
                    lastMacro = runningMacro;
                    runningMacro = 0;
                }
                break;
        }
    }

    public void update(double runTime) {
        drive.update();
        slides.update(runTime);
        arm.update();
    }

    public String getTelemetry() {
        return String.format("Arm:\n------------\n%s\nSlides:\n------------\n%s", arm.getTelemetry(), slides.getTelemetry());
    }
}