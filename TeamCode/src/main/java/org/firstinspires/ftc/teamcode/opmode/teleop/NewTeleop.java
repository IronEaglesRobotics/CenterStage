package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.hardware.DoorPos.DoorPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.DoorPos.DoorPosition.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.robby.Slides;

@Config
@TeleOp(name = "Main TeleOp", group = "OpModes")
public class NewTeleop extends OpMode {

    public static double normal = 0.5;
    public static double turbo = 1;

    private Robot robot;
    private Controller controller1;
    private Controller controller2;

    @Override
    public void init() {
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        this.robot = new Robot(hardwareMap);
        robot.intake.setpos(Intake.Position.STACK1);

        telemetry.addLine("Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Driver 1
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double z = -gamepad1.right_stick_x;

        if (controller1.getA().isPressed()) {
            x *= turbo;
            y *= turbo;
            z *= turbo;
        } else {
            x *= normal;
            y *= normal;
            z *= normal;
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));

        // Driver 2
        robot.intake.setDcMotor(gamepad2.right_trigger);
        if (controller2.getRightBumper().isJustPressed()) {
            robot.intake.incrementPos();
        }
        if (controller2.getLeftBumper().isJustPressed()) {
            robot.intake.decrementPos();
        }

        // macros
        switch (robot.runningMacro) {
            case (0): // manual mode
                robot.slides.increaseTarget(controller2.getLeftStick().getY());
                if (controller2.getX().isJustPressed()) {
                    robot.runningMacro = 1;
                } else if (controller2.getY().isJustPressed()) {
                    robot.runningMacro = 2;
                } else if (controller2.getB().isJustPressed()) {
                    robot.runningMacro = 3;
                } else if (controller2.getA().isJustPressed()) {
                    robot.runningMacro = 4;
                }
                if (robot.intake.getPower() >= 0.01) {
                    robot.arm.setDoor(OPEN);
                } else {
                    robot.arm.setDoor(CLOSE);
                }
                break;
            case (1):
                robot.extendMacro(Slides.Position.TIER1, getRuntime());
                break;
            case (2):
                robot.extendMacro(Slides.Position.TIER2, getRuntime());
                break;
            case (3):
                robot.extendMacro(Slides.Position.TIER3, getRuntime());
                break;
            case (4):
                robot.resetMacro(Slides.Position.DOWN, getRuntime());
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
