package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CenterStage", group = "Development")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private double speedMultiplier = 1.0; // Start at 100% speed
    private MecanumDrive drive;

    @Override
    public void init() {
        this.robot = new Robot().init(hardwareMap);
        this.robot.getHang().lock();
    }

    @Override
    public void loop() {
//Drive
        robot.getDrive().setInput(gamepad1, gamepad2);
//Hang
        if (gamepad1.b) {
            this.robot.getHang().release();
        } else {
            this.robot.getHang().lock();
        }
//Intake
        if (gamepad1.x) {
            this.robot.getIntake().spinIn();
        } else if (gamepad1.y) {
            this.robot.getIntake().spinOut();
        }



    }
}