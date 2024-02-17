package org.firstinspires.ftc.teamcode.opmodes;

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

    @Override
    public void loop() {
        boolean slideUp = gamepad2.dpad_up;
        boolean restArm = gamepad2.x;
        boolean pickupArm = gamepad2.dpad_down;
        boolean scoreArm = gamepad2.a;
        boolean plane = gamepad2.right_bumper;
        boolean claw = gamepad2.b;
        boolean pickupWrist = gamepad2.x;
        boolean scoreWrist = gamepad2.a;
        boolean slideDown = gamepad2.dpad_left;
        boolean hang = gamepad2.left_bumper;
        boolean hangRelease = gamepad2.y;
        boolean hangPlane = gamepad2.dpad_right;
//Drive
        robot.getDrive().setInput(gamepad1, gamepad2);
//slides
        if (slideUp) {
            this.robot.getSlides().slideUp();
        } else if (slideDown) {
            this.robot.getSlides().slideDown();
        } else {
            this.robot.getSlides().slideStop();
        }
//Arm
        if (pickupArm) {
            this.robot.getArm().pickup();
        } else if (restArm) {
            this.robot.getArm().armRest();
        } else if (scoreArm) {
            this.robot.getArm().armSecondaryScore();
        }
//Claw
        if (claw) {
            this.robot.getClaw().open();
            this.robot.getLed().white();
        } else {
            this.robot.getClaw().close();
            this.robot.getLed().gold();
        }
//Wrist
        if (pickupWrist) {
            this.robot.getWrist().wristPickup();
        } else if (scoreWrist) {
            this.robot.getWrist().wristScore();
        }
//Hang
        if (hang) {
            this.robot.getHang().hang();
        } else if (hangRelease){
            this.robot.getHang().hangRelease();
        } else if (hangPlane) {
            this.robot.getHang().hangPlane();
        } else {
            this.robot.getHang().hangIdle();
        }
        int Position = this.robot.getHang().hangRight.getCurrentPosition();
        telemetry.addData("position",(Position));
        telemetry.update();
//Plane
        if (plane) {
            this.robot.getPlane().planeLaunch();
        }else {
            this.robot.getPlane().planeLock();
        }
    }
}