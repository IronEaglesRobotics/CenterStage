package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config

@Autonomous (name = "Start From Left Center Spike Test", group = "Testing", preselectTeleOp = "KhangMain")

public class TestAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        this.robot = new Robot(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //bot is 15 in and a half so middle is 7.75
        //17.8 in so half is 8.9 in
        Pose2d startPOS = new Pose2d(-63.1, -36.75, Math.toRadians(90));
    }
}
