package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_ARM_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_LIFT_DELTA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private double clawArmPosition = 0;
    private boolean screwArmPos = false;
    private boolean previousScrewArmToggle = false;
    private boolean previousScrewDeposit = false;
    private boolean previousScrewIntake = false;
    private boolean previousScrewReset = false;
    private boolean previousSlideUp = false;
    private boolean previousSlideDown = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.robot = new Robot(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    // Gamepad 2 - Arm Control
    // RT - Hanger Raise
    // RB - Hanger Lift Robot
    // DPad Left/Right - Gantry Left/Right
    // DPad Up/Down - Lift Up/Down
    @Override
    public void loop() {
        // Drive
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;
        this.robot.getDrive().setInput(x, y, z);


        // Claw
        boolean openClaw = false;
        boolean clawUp = false;
        boolean clawDown = false;
        if (openClaw) {
            this.robot.getClaw().open();
        } else {
            this.robot.getClaw().close();
        }
        if (clawUp) {
            this.clawArmPosition += CLAW_ARM_DELTA;
            this.robot.getClaw().setArmPosition(clawArmPosition);
        } else if (clawDown) {
            this.clawArmPosition -= CLAW_ARM_DELTA;
            this.robot.getClaw().setArmPosition(clawArmPosition);
        }

        // Robot Lift
        boolean raiseRobotLift = false;
        boolean liftRobot = false;
        if (raiseRobotLift) {
            this.robot.getLift().raise();
        } else if (liftRobot) {
            this.robot.getLift().lift();
        }

        // Gantry
        boolean screwArmToggle = false;
        boolean screwDeposit = false;
        boolean screwIntake = false;
        boolean screwReset = false;
        boolean slideUp = false;
        boolean slideDown = false;
        if (!previousScrewArmToggle && screwArmToggle) {
            if (screwArmPos) {
                this.robot.getGantry().armOut();
            } else {
                this.robot.getGantry().armIn();
            }
            screwArmPos = !screwArmPos;
        }
        if (!previousScrewDeposit && screwDeposit) {
            this.robot.getGantry().deposit();
        } else if (!previousScrewIntake && screwIntake) {
            this.robot.getGantry().intake();
        } else if (screwReset) {
            this.robot.getGantry().resetScrew();
        }
        if ((!previousSlideUp && slideUp) || (!previousSlideDown && slideDown)) {
            int currentPosition  = this.robot.getGantry().getSlidePosition();
            this.robot.getGantry().setTarget(currentPosition + GANTRY_LIFT_DELTA);
        }

        this.previousSlideUp = slideUp;
        this.previousScrewArmToggle = screwArmToggle;
        this.previousScrewDeposit = screwDeposit;
        this.previousScrewIntake = screwIntake;
        this.previousScrewReset = screwReset;
    }
}
