package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp", group = "Development")
public class MainTeleOp extends OpMode {
    GamepadEx controller1;
    GamepadEx controller2;
    private Robot robot;
    private MecanumDrive drive;

    @Override
    public void init() {
        this.robot = new Robot().init(hardwareMap);
        this.controller2 = new GamepadEx(this.gamepad2);
        this.controller1 = new GamepadEx(this.gamepad1);
    }

    public void loop() {
//GamePad Controls
        boolean slideUp = controller2.isDown(GamepadKeys.Button.DPAD_UP);
        boolean slideDown = controller2.isDown(GamepadKeys.Button.DPAD_LEFT);
        boolean hang = gamepad2.left_bumper;
        boolean hangRelease = gamepad2.right_bumper;
        boolean hangPlane = gamepad2.y;
        boolean plane = gamepad2.dpad_right;
//Read Controller
        controller1.readButtons();
        controller2.readButtons();
//Drive
        robot.getDrive().setInput(gamepad1, gamepad2);
//slides
        if (slideUp) {
            this.robot.getSlides().slideUp();
        } else if (slideDown) {
            this.robot.getSlides().slideDown();
        } else if (controller2.wasJustReleased(GamepadKeys.Button.DPAD_UP)
                || controller2.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            this.robot.getSlides().slideStop();
        }
        if (gamepad2.left_trigger > .1) {
            Robot.Slides.SLIDE_POWER_UP = .3;
        } else {
            Robot.Slides.SLIDE_POWER_UP = .7;
        }
////Macros
        this.robot.pickupMacro(controller2, getRuntime()); //DPADDOWN
        this.robot.armMacro(controller2, getRuntime()); //X
//
//Arm and Wrist
//        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            this.robot.getArm().pickup(true);
//        } else
//        if (controller2.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
//            this.robot.getArm().armRest(true);
//        } else
        if (controller2.wasJustPressed(GamepadKeys.Button.A)) {
            this.robot.getArm().armRest();
            this.robot.getWrist().wristPickup();
        }
//Claw
        if (controller2.wasJustPressed(GamepadKeys.Button.B)) {
            gamepad1.rumble(300);
        } else if (controller2.isDown(GamepadKeys.Button.B) || controller2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            this.robot.getClaw().open();
        } else if (controller2.wasJustReleased(GamepadKeys.Button.B)) {
            this.robot.getClaw().close();
        }
//Hang
        if (hang) {
            this.robot.getHang().hang();
        } else if (hangRelease) {
            this.robot.getHang().hangRelease();
        } else if (hangPlane) {
            this.robot.getHang().hangPlane();
        }
//Plane
        if (plane) {
            this.robot.getPlane().planeLaunch();
        } else {
            this.robot.getPlane().planeLock();
        }
//Telemetry
        int PositionLeft = this.robot.getSlides().slidesLeft.getCurrentPosition();
        telemetry.addData("positionLeft", (PositionLeft));
        int PositionRight = this.robot.getSlides().slidesRight.getCurrentPosition();
        telemetry.addData("positionRight", (PositionRight));
        telemetry.update();
//Update Robot
        this.robot.update();
    }
}
