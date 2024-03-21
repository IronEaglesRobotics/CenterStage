package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Intake.Position.UP;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Config
public class Robot {

    //Create each object
    public SampleMecanumDrive drive;
    public Camera camera;
    public Intake intake;
    public Slides slides;
    public Arm arm;
    public LEDs leds;
    public endGame_Mechs endGameMechs;

    public double macroStartTime = 0;
    public int macroState = 0;
    public int runningMacro = 0; // 0 = no macro | 1 = tier 1 | 2 = tier 2 | 3 = tier 3 | 4 = return
    public int lastMacro = 0;

    private boolean camEnabled = true;
    public static double slideWait = 1.5;
    public static double scoreWait = 0.65;
    public static double armWait = 2.0;

    public int alliance = 0;

    //Name the objects
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap, UP);
        slides = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);
        endGameMechs = new endGame_Mechs(hardwareMap);
        camEnabled = true;
        leds = new LEDs(hardwareMap);
    }

    public void extendMacro(int slidePos, double runTime) {
        if (alliance == 1){
            leds.setPattern(2);
        }else{
            leds.setPattern(3);
        }

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
                    if (alliance == 1){
                        leds.setPattern(93);
                    }else{
                        leds.setPattern(80);
                    }
                }
                break;
        }
    }

    public void resetMacro(int slidePos, double runTime) {
        if (alliance == 1){
            leds.setPattern(2);
        }else{
            leds.setPattern(3);
        }

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
                    if (alliance == 1){
                        leds.setPattern(93);
                    }else{
                        leds.setPattern(80);
                    }
                }
                break;
        }
    }
    public void extendMacro_auto(int slidePos, double runTime, int slidepos2 ) {
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
                    macroState++;
                }
                break;

            case (4):
                slides.setTarget(slidepos2);
                break;
        }
    }


    public void resetMacro_auto(int slidePos2, double runTime, int slidePos) {
        switch(macroState) {
            case(0):
                macroStartTime = runTime;
                arm.setDoor(OPEN);
                macroState++;
                break;
            case(1):
                if (runTime > macroStartTime + scoreWait) {
                    slides.setTarget(slidePos);
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
                slides.setTarget(slidePos2);
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
//        drive.getPoseEstimate().getX();
//         = estimatedPose.getX();
  Pose2d estimatedPose = null;
//        if (camera != null) {
//            estimatedPose = this.camera.estimatePoseFromAprilTag();
//        }
        //drive.update(estimatedPose);

//        Pose2d p1 = drive.getPoseEstimate();
//
//        Pose2d p2 = this.camera.estimatePoseFromAprilTag();
//
//        double D = Math.sqrt(Math.pow((p2.getX()-p1.getX()),2) + Math.pow((p2.getY()-p1.getY()),2));
//
//        if (D >= 6 || D <= 1){
//            estimatedPose = null;
//        }else{
//            estimatedPose = this.camera.estimatePoseFromAprilTag();
//
//        }

        drive.update(/*estimatedPose*/);
        slides.update(runTime);
        arm.update();
        //leds.setPattern();
    }



    public String getTelemetry() {
        return String.format("Arm:\n------------\n%s\nSlides:\n------------\n%s\nIMU:\n------------\n%s" , arm.getTelemetry(), slides.getTelemetry(), drive.getExternalHeadingVelocity());
    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        return this.drive.trajectorySequenceBuilder(this.drive.getPoseEstimate());
    }
}