package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.CenterStageCommon;
import org.firstinspires.ftc.teamcode.vision.Detection;

public abstract class AutoBase extends LinearOpMode {
    protected Robot robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected CenterStageCommon.PropLocation propLocation;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
        this.dashboard = FtcDashboard.getInstance();
        this.dashboardTelemetry = dashboard.getTelemetry();

        this.robot.getCamera().setAlliance(CenterStageCommon.Alliance.Red);

        while(!isStarted() && !isStopRequested()) {
            this.propLocation = this.robot.getCamera().getPropLocation();
            Detection detection = this.robot.getCamera().getProp();
            this.dashboardTelemetry.addData("Prop", detection.getCenterPx());
            this.dashboardTelemetry.addData("Prop Location", this.propLocation);
            this.dashboardTelemetry.update();
        }
    }
}
