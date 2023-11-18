package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "Left Auto", preselectTeleOp = "MainTeleOp")
public class BlueFrontStage extends AutoBase {
    public BlueFrontStage() {
        super(
                CenterStageCommon.Alliance.Blue,
                new Pose2d(-36, 63, Math.toRadians(-90)),
                new Pose2d(-36, 11),
                new Pose2d(62, 12));
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
