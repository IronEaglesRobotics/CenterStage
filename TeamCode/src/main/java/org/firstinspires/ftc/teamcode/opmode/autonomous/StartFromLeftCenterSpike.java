package org.firstinspires.ftc.teamcode.opmode.autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Start From Left Wall", group = "Left Start", preselectTeleOp = "Khang Main")
public class StartFromLeftCenterSpike extends AbstractAuto {

    //set to public Drive to revert
    public SampleMecanumDrive drive;
    public Robot robot;
    public Camera camera;
    private boolean camEnabled = true;
    public CameraPosition cameraPosition;
    private int teamElementLocation = 2;
    //Steps
    public Trajectory start;
    public Trajectory step1;
    public Trajectory step2;

    @Override
    public void setAlliance() {}

    @Override
    public void setCameraPosition() {
        cameraPosition = CameraPosition.CENTER;
    }

    @Override
    public boolean useCamera() {
        return true;
    }

    public void Robot(HardwareMap hardwareMap) {
        //set to new Drive to revert
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap);
        camera.initTargetingCamera();
        camEnabled = true;
    }

    @Override
    public void makeTrajectories() {

        // positions
        Pose2d start = new Pose2d(-65.125,-48,Math.toRadians(-90));
        Pose2d step1 = new Pose2d(-24,-48,Math.toRadians(0));
        Pose2d step2 = new Pose2d(-24,-48,Math.toRadians(0));

        this.start = robot.drive.trajectoryBuilder(start)
                .lineToSplineHeading(step1,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }
}
