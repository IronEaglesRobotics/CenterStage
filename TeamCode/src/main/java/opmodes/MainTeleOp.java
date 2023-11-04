package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_ARM_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_LIFT_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_SCREW_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_X_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.PICKUP_ARM_MAX;

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
    private boolean previousScrewReset = false;
    private boolean previousSlideUp = false;
    private boolean previousSlideDown = false;
    private double currentScrewPosition = 1f;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.clawArmPosition = PICKUP_ARM_MAX;

        this.robot = new Robot(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = -gamepad1.right_stick_x;
        this.robot.getDrive().setInput(0, y, z);

        this.telemetry.addLine(this.robot.getDrive().getTelemetry());
        this.telemetry.update();

        // Button Mappings
        boolean openClaw = gamepad2.b; // B
        boolean clawUp = gamepad2.y; // Y
        boolean clawDown = gamepad2.a; // A

        boolean raiseRobotLift = gamepad2.right_trigger > 0.25; // RT
        boolean liftRobot = gamepad2.right_bumper; // RB

        boolean screwArmToggle = gamepad2.x; // X
        boolean screwDeposit = gamepad2.left_trigger > 0.25; // LT
        boolean screwReset = gamepad2.left_bumper; // LB
        boolean slideUp = gamepad2.left_stick_y < -0.25; // Left Y Axis Joystick
        boolean slideDown = gamepad2.left_stick_y > 0.25; // Left Y Axis Joystick
        boolean gantryLeft = gamepad2.dpad_left; // dpad left
        boolean gantryRight = gamepad2.dpad_right; // dpad right

        // Claw
        if (openClaw) {
            this.robot.getClaw().open();
        } else if (!clawUp && !clawDown){
            this.robot.getClaw().close();
        }
        if (clawUp) {
            this.clawArmPosition = Math.min(1, this.clawArmPosition + CLAW_ARM_DELTA);
            this.robot.getClaw().setArmPosition(clawArmPosition);
        } else if (clawDown) {
            this.robot.getClaw().open();
            this.clawArmPosition = Math.max(0, this.clawArmPosition - CLAW_ARM_DELTA);
            this.robot.getClaw().setArmPosition(clawArmPosition);
        }

/*

        // Gantry
        if (!previousScrewArmToggle && screwArmToggle) {
            if (screwArmPos) {
                this.robot.getGantry().armOut();
            } else {
                this.robot.getGantry().armIn();
            }
            screwArmPos = !screwArmPos;
        }
        if (!previousScrewDeposit && screwDeposit) {
            this.currentScrewPosition += GANTRY_SCREW_DELTA;
            this.robot.getGantry().setScrew(currentScrewPosition);
        } else if (!previousScrewReset && screwReset) {
            this.robot.getGantry().resetScrew();
        }
        if ((!previousSlideUp && slideUp) || (!previousSlideDown && slideDown)) {
            int currentPosition  = this.robot.getGantry().getSlidePosition();
            this.robot.getGantry().setTarget(currentPosition + GANTRY_LIFT_DELTA);
        }
        if (gantryLeft) {
            this.robot.getGantry().setX(this.robot.getGantry().getX() + GANTRY_X_DELTA);
        } else if (gantryRight) {
            this.robot.getGantry().setX(this.robot.getGantry().getX() - GANTRY_X_DELTA);
        }

        // Robot Lift
        if (raiseRobotLift) {
            this.robot.getLift().raise();
        } else if (liftRobot) {
            this.robot.getLift().lift();
        }
 */
        this.previousSlideUp = slideUp;
        this.previousScrewArmToggle = screwArmToggle;
        this.previousScrewDeposit = screwDeposit;
        this.previousScrewReset = screwReset;

    }
}
