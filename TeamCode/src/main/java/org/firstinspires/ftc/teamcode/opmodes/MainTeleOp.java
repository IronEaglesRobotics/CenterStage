package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp", group = "Development")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private MecanumDrive drive;

    @Override
    public void init() {
        this.robot = new Robot().init(hardwareMap);
    }

    GamepadEx controller2 = new GamepadEx(gamepad2);
    GamepadEx controller1 = new GamepadEx(gamepad1);


    public void loop() {
//        boolean slideUp = controller2.isDown(GamepadKeys.Button.DPAD_UP);
//        boolean slideDown = controller2.isDown(GamepadKeys.Button.DPAD_LEFT);
//        boolean hang = gamepad2.left_bumper;
//        boolean hangRelease = gamepad2.right_bumper;
//        boolean hangPlane = gamepad2.y;
//        boolean plane = gamepad2.dpad_right;
//Drive
        robot.getDrive().setInput(controller1, controller2);
//slides
//        if (slideUp) {
//            this.robot.getSlides().slideUp();
//        } else if (slideDown) {
//            this.robot.getSlides().slideDown();
//        } else if (controller2.wasJustReleased(GamepadKeys.Button.DPAD_UP)
//                || controller2.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
//            this.robot.getSlides().slideStop();
//        }
////Macros
//        this.robot.pickupMacro(controller2, getRuntime()); //DPADDOWN
//
////Arm and Wrist
//        if (controller2.wasJustPressed(GamepadKeys.Button.X)) {
//            this.robot.getArm().armSecondaryScore();
//            this.robot.getWrist().wristScore();
//        } else if (controller2.wasJustPressed(GamepadKeys.Button.A)) {
//            this.robot.getArm().armRest();
//            this.robot.getWrist().wristPickup();
//        }
////Claw
//        if (controller2.wasJustPressed(GamepadKeys.Button.B)) {
//            gamepad1.rumble(300);
//        } else if (controller2.isDown(GamepadKeys.Button.B)){
//            this.robot.getClaw().open();
//        } else if (controller2.wasJustReleased(GamepadKeys.Button.B)){
//            this.robot.getClaw().close();
//        }
////Hang
//        if (hang) {
//            this.robot.getHang().hang();
//        } else if (hangRelease){
//            this.robot.getHang().hangRelease();
//        } else if (hangPlane) {
//            this.robot.getHang().hangPlane();
//        }
//
//        int Position = this.robot.getHang().hangRight.getCurrentPosition();
//        telemetry.addData("position",(Position));
//        telemetry.update();
//
////Plane
//        if (plane) {
//            this.robot.getPlane().planeLaunch();
//        }else {
//            this.robot.getPlane().planeLock();
//        }
//
    }
}
