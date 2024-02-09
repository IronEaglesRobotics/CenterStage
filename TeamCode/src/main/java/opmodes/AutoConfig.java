package opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.CenterStageCommon;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@AllArgsConstructor
@NoArgsConstructor
@Builder
@Data
public class AutoConfig {
    public static int DEPOSIT_HEIGHT = 100;
    public static double SCORING_DURATION_S = 2f;
    public static double APRIL_TAG_RIGHT_DELTA = -8.5;
    public static double APRIL_TAG_LEFT_DELTA = 7.0;

    private Pose2d initialPosition;
    private Pose2d leftParkPosition;
    private Pose2d rightParkPosition;
    private ParkLocation parkLocation = ParkLocation.Left;
    private long delay;

    public Pose2d getSelectionParkPosition() {
        return this.parkLocation == ParkLocation.Left
                ? this.leftParkPosition
                : this.rightParkPosition;
    }

    protected CenterStageCommon.Alliance getAlliance() {
        if (this.initialPosition == null) {
            throw new RuntimeException("Thou fool! Thou must set the initial position!");
        }
        return this.getInitialPosition().getY() > 0
                ? CenterStageCommon.Alliance.Blue
                : CenterStageCommon.Alliance.Red;
    }

    public void increaseDelay() {
        this.delay += 1000;
    }

    public void decreaseDelay() {
        this.delay -= 1000;
    }

    protected boolean isBackstage() {
        return this.initialPosition.getX() > 0;
    }

    public enum ParkLocation {
        Left,
        Right
    }
}
