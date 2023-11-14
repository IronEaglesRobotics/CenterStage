//package org.firstinspires.ftc.teamcode.opmode.autonomous;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.hardware.Camera;
//import org.firstinspires.ftc.teamcode.hardware.Intake;
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.util.CameraPosition;
//
//@Autonomous(name = "Start From Left Wall", group = "Left Start", preselectTeleOp = "Khang Main")
//public class StartFromLeftCenterSpike extends AbstractAuto {
//
//    public Robot robot;
//    public CameraPosition cameraPosition;
//    //Steps
//    public Trajectory start;
//
//    @Override
//    public void initializeSteps(int location) {
//        followTrajectory(start);
//    }
//
//    @Override
//    public void setAlliance() {}
//
//    @Override
//    public void setCameraPosition() {
//        cameraPosition = CameraPosition.FRONT;
//    }
//
//    @Override
//    public boolean useCamera() {
//        return true;
//    }
//
//    @Override
//    public void waitForStart() {
//        super.waitForStart();
//    }
//
//    @Override
//    public void makeTrajectories() {
//
//        // positions
//        Pose2d start = new Pose2d(-65.125,-48,Math.toRadians(90));
//        Vector2d step1 = new Vector2d(-24,-48);
//        Pose2d step2 = new Pose2d(-24,-48,Math.toRadians(90));
//
////        this.start = robot.drive.trajectoryBuilder(start)
//        this.start = robot.drive.trajectoryBuilder(new Pose2d())
//                .forward(3)
//                .build();
//    }
//}
