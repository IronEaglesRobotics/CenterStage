package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.CLAW_ARM_DELTA;
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
    private boolean previousSlideUp = false;
    private boolean previousSlideDown = false;
    private boolean previousRobotLiftReset = false;
    private boolean previousRobotLiftExtend = false;
    private boolean liftArmShouldBeUp = false;
    private boolean screwArmIsMoving = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.clawArmPosition = PICKUP_ARM_MAX;

        this.robot = new Robot(this.hardwareMap, telemetry);
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

        boolean robotLiftRotation = gamepad2.right_trigger > 0.05; // RT
        boolean robotLiftExtend = gamepad2.right_trigger > 0.5; // RT
        boolean robotLiftReset = gamepad2.right_stick_button;

        boolean screwArmToggle = gamepad2.x; // X
        boolean screwDeposit = gamepad2.left_trigger > 0.25; // LT
        boolean screwIntake = gamepad2.left_bumper; // LB
        boolean slideUp = gamepad2.left_stick_y < -0.25; // Left Y Axis Joystick
        boolean slideDown = gamepad2.left_stick_y > 0.25; // Left Y Axis Joystick
        boolean gantryLeft = gamepad2.dpad_left; // dpad left
        boolean gantryRight = gamepad2.dpad_right; // dpad right

        // Claw
        if (openClaw) {
            this.robot.getClaw().open();
            this.screwArmIsMoving = false;
        } else if (!clawUp && !clawDown && !this.screwArmIsMoving){
            this.robot.getClaw().close();
        }
        if (clawUp) {
            this.screwArmIsMoving = false;
            this.clawArmPosition = Math.min(1, this.clawArmPosition + CLAW_ARM_DELTA);
            this.robot.getClaw().setArmPosition(clawArmPosition);
        } else if (clawDown) {
            this.screwArmIsMoving = false;
            this.robot.getClaw().open();
            this.clawArmPosition = Math.max(0, this.clawArmPosition - CLAW_ARM_DELTA);
            this.robot.getClaw().setArmPosition(clawArmPosition);
        }

        // Gantry
        if (!previousScrewArmToggle && screwArmToggle) {
            this.screwArmIsMoving = true;
            if (screwArmPos) {
                this.robot.getGantry().armIn();
            } else {
                this.robot.getClaw().open();
                this.robot.getGantry().armOut();
            }
            this.robot.getClaw().open();
            screwArmPos = !screwArmPos;
        }
        if (screwDeposit) {
            this.robot.getGantry().deposit();
        } else if (screwIntake) {
            this.robot.getGantry().intake();
        } else {
            this.robot.getGantry().stop();
        }
//        if ((!previousSlideUp && slideUp) || (!previousSlideDown && slideDown)) {
//            int currentPosition  = this.robot.getGantry().getSlidePosition();
//            this.robot.getGantry().setTarget(currentPosition + GANTRY_LIFT_DELTA);
//        }
//
//        if (gantryLeft) {
//            this.robot.getGantry().setX(this.robot.getGantry().getX() + GANTRY_X_DELTA);
//        } else if (gantryRight) {
//            this.robot.getGantry().setX(this.robot.getGantry().getX() - GANTRY_X_DELTA);
//        }

        // Robot Lift

        if (robotLiftRotation || this.liftArmShouldBeUp) {
            this.liftArmShouldBeUp = true;
            if (this.robot.getGantry().isIn()) {
                this.robot.getGantry().armOut();
            } else {
                if (robotLiftExtend) {
                    this.robot.getLift().extend();
                }
                this.robot.getLift().up();
            }
        }

        if (!robotLiftExtend && previousRobotLiftExtend) {
            this.robot.getLift().retract();
        }

        if (robotLiftReset) {
            this.robot.getLift().startReset();
        } else if (previousRobotLiftReset) {
            this.liftArmShouldBeUp = false;
            this.robot.getLift().stopReset();
        }

        this.robot.update();

        this.previousSlideUp = slideUp;
        this.previousScrewArmToggle = screwArmToggle;
        this.previousRobotLiftReset = robotLiftReset;
        this.previousRobotLiftExtend = robotLiftExtend;
    }
}
