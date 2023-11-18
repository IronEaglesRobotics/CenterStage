package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

@Autonomous(name = "Left Auto", preselectTeleOp = "MainTeleOp")
public class LeftAuto extends AutoBase {
    public LeftAuto() {
        super(
                CenterStageCommon.Alliance.Blue,
                new Pose2d(-36, 63, Math.toRadians(-90)),
                new Vector2d(-36, 11));
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
