package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.Arm.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;

@Config
@TeleOp(name = "Main TeleOp", group = "OpModes")
public class MainTeleOp extends OpMode {

    public static double normal = 0.7;
    public static double turbo = 1;
    public static double slow_mode = 0.35;
    public static double intakeMax = 0.65;
    public static double intakeMax2 = -0.65;

    private Robot robot;
    private Controller controller1;
    private Controller controller2;

    @Override
    public void init() {
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        this.robot = new Robot(hardwareMap);
//        robot.intake.setpos(Intake.Position.STACK1);
        this.robot.endGameMechs.hold();
//        while (robot.camera.getFrameCount() < 1) {
//            telemetry.addLine("Initializing...");
//            telemetry.update();
//        }

        telemetry.addLine("Initialized");
        telemetry.update();


    }

    @Override
    public void loop() {
        // Driver 1
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double z = -gamepad1.right_stick_x;

        if (controller1.getRightTrigger().getValue() > 0.1) {
            x *= turbo;
            y *= turbo;
            z *= turbo;
        }
        else if (controller1.getLeftTrigger().getValue() > 0.1) {
            x *= slow_mode;
            y *= slow_mode;
            z *= slow_mode;
        } else {
            x *= normal;
            y *= normal;
            z *= normal;
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));

        // Driver 2
        if (controller2.getRightTrigger().getValue()>=0.1){
            robot.intake.setDcMotor(controller2.getRightTrigger().getValue()*intakeMax);
        }
        else if(controller2.getLeftTrigger().getValue()>=0.1){
            robot.intake.setDcMotor(controller2.getLeftTrigger().getValue()*intakeMax2);
        }
        else {
            robot.intake.setDcMotor(0);
        }

        if (controller2.getRightBumper().isJustPressed()) {
            robot.intake.incrementPos();
        }
        if (controller2.getLeftBumper().isJustPressed()) {
            robot.intake.decrementPos();
        }

        // Drone launcher
        if (controller1.getA().isPressed() && !controller1.getStart().isPressed() && !controller2.getStart().isPressed()) {
            this.robot.endGameMechs.launch();
        } else {
            this.robot.endGameMechs.reset();
        }
        //Hang Servos
        if (controller1.getX().isPressed() && !controller1.getStart().isPressed() && !controller2.getStart().isPressed()) {
            this.robot.endGameMechs.release();
        }


        // macros
        switch (robot.runningMacro) {
            case (0): // manual mode

                robot.slides.increaseTarget(controller2.getLeftStick().getY());
                if (robot.intake.getPower() >= 0.01) {
                    robot.arm.setDoor(OPEN);
                } else if (robot.intake.getPower() <= -0.01) {
                    robot.arm.setDoor(OPEN);
                } else if (controller2.getLeftBumper().isPressed()) {
                    robot.arm.setDoor(Arm.DoorPosition.OPEN);
                } else {
                    robot.arm.setDoor(CLOSE);
                }

                if (controller2.getX().isJustPressed()) {
                    robot.runningMacro = 1;
                } else if (controller2.getY().isJustPressed()) {
                    robot.runningMacro = 2;
                } else if (controller2.getB().isJustPressed() && !controller1.getStart().isPressed() && !controller2.getStart().isPressed()) {
                    robot.runningMacro = 3;
                } else if (controller2.getA().isJustPressed() && !controller1.getStart().isPressed() && !controller2.getStart().isPressed()) {
                    robot.runningMacro = 4;
                } else if (controller2.getDDown().isJustPressed() ) {
                    robot.runningMacro = 5;
                }

                break;
            case (1):
                robot.extendMacro(Slides.tier1, getRuntime());
                break;
            case (2):
                robot.extendMacro(Slides.tier2, getRuntime());
                break;
            case (3):
                robot.extendMacro(Slides.tier3, getRuntime());
                break;
            case (4):
                robot.resetMacro(0, getRuntime());
                break;
            case(5):
                robot.extendMacro(Slides.mini_tier1, getRuntime());
                break;
        }

        // update and telemetry
        robot.update(getRuntime());
        controller1.update();
        controller2.update();
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
