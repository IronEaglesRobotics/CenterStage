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
        boolean hang = gamepad2.dpad_up;
        boolean restArm = gamepad2.dpad_left || gamepad2.x;
        boolean pickupArm = gamepad2.dpad_down;
        boolean scoreArm = gamepad2.dpad_right || gamepad2.a;
        boolean accurateScoreArm = gamepad2.y;
        boolean claw = gamepad2.b;
        boolean pickupWrist = gamepad2.left_bumper || gamepad2.x;
        boolean scoreWrist = gamepad2.right_bumper || gamepad2.a;
//Drive
        robot.getDrive().setInput(gamepad1, gamepad2);
//Hang
        if (hang) {
            this.robot.getHang().release();
        }
//Arm
        if (pickupArm) {
            this.robot.getArm().pickup();
        } else if (restArm) {
            this.robot.getArm().armRest();
        } else if (scoreArm) {
            this.robot.getArm().armScore();
        } else if (accurateScoreArm) {
            this.robot.getArm().armAccurateScore();
        }
//Claw
        if (claw) {
            this.robot.getClaw().open();
        } else {
            this.robot.getClaw().close();
        }
//Wrist
        if (pickupWrist) {
            this.robot.getWrist().wristPickup();
        } else if (scoreWrist) {
            this.robot.getWrist().wristScore();
        }
    }
}