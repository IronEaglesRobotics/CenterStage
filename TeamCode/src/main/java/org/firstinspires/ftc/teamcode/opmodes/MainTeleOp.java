package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Configurables;

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
//Drive
        robot.getDrive().setInput(gamepad1, gamepad2);
//Hang
        if (gamepad2.dpad_up) {
            this.robot.getHang().release();
        } else {
            this.robot.getHang().lock();
        }
//Intake
//        if (gamepad1.x) {
//            this.robot.getIntake().spinIn();
//        } else if (gamepad1.y) {
//            this.robot.getIntake().spinOut();
//        } else {
//            this.robot.getIntake().stop();
//        }
//Arm
        if (gamepad2.dpad_down) {
            this.robot.getArm().pickup();
        } else if (gamepad2.dpad_right ||gamepad2.x) {
            this.robot.getArm().scoreArm();
        } else if (gamepad2.dpad_left || gamepad2.a) {
            this.robot.getArm().rest();
        }
//Claw
        if (gamepad2.b) {
            this.robot.getClaw().open();
        }  else if (gamepad2.y) {
            this.robot.getClaw().openScore();
        } else {
            this.robot.getClaw().close();
        }
//Wrist
        if (gamepad2.left_bumper || gamepad2.x) {
            this.robot.getWrist().wristPickup();
        } else if (gamepad2.right_bumper || gamepad2.a) {
            this.robot.getWrist().wristScore();
        }
//SLOWMO
        if (gamepad1.y) {
            Configurables.SPEED = .5;
            Configurables.TURN = .75;
        } else {
            Configurables.SPEED = 1;
            Configurables.TURN = 1;
        }
    }
}