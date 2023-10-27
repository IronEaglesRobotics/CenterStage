package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Khang Main", group = "OpModes")
public class KhangMain extends OpMode {

    private Intake.Position Currentpos = Intake.Position.UP;
    private Robot robot;
    private Controller controller2;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        controller2 = new Controller(gamepad2);
    }

    @Override
    public void loop() {
//        this.robot.getDrive().setInput(gamepad1, gamepad2);
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        robot.intake.setDcMotor(gamepad2.right_trigger);
        robot.intake.setpos(Currentpos);
        controller2.update();

        if (controller2.getLeftBumper().isJustPressed()) {
            Currentpos = Currentpos.nextPosition();
        }

        if (controller2.getRightBumper().isJustPressed()) {
            Currentpos = Currentpos.previousPosition();
        }



        // Read pose
//        Pose2d poseEstimate = drive.getPoseEstimate();
//
//// Create a vector from the gamepad x/y inputs
//// Then, rotate that vector by the inverse of that heading
//        Vector2d input = new Vector2d(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x
//        ).rotated(-poseEstimate.getHeading());
//
//// Pass in the rotated input + right stick value for rotation
//// Rotation is not part of the rotated input thus must be passed in separately
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        -gamepad1.right_stick_x
//                )
//        );
    }
}
