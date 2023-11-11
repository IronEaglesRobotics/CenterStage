import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AbstractAuto;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Red Right Park", group = "Right Start", preselectTeleOp = "Khang Main")
public class parkAutoRed extends AbstractAuto {
    public SampleMecanumDrive drive;
    public Robot robot;
    public Camera camera;
    private boolean camEnabled = true;
    public CameraPosition cameraPosition;
    public Trajectory start;

    @Override
    public void setAlliance() {}

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
        Pose2d start = new Pose2d(65.125,6,Math.toRadians(180));
        drive.setPoseEstimate(start);

        Trajectory park = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(65.125,60), Math.toRadians(180))
                .build();

        drive.followTrajectory(park);
    }
}
