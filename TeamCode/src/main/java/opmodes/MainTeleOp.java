package opmodes;

import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.CLAW_ARM_DELTA;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.PICKUP_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.SLIDE_UP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.X_CENTER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.X_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConfig.X_MIN;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private FtcDashboard dashboard;

    @Override
    public void init() {
        this.dashboard = FtcDashboard.getInstance();
        this.clawArmPosition = PICKUP_ARM_MAX;

        this.robot = new Robot(this.hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive
        boolean slowmode = gamepad1.right_bumper || gamepad1.y;
        this.robot.getDrive().setInput(gamepad1, gamepad2, slowmode);

        // Button Mappings
        // Claw / Pickup
        boolean openClaw = gamepad2.b; // B
        boolean clawUp = gamepad2.y; // Y
        boolean clawDownSafe = gamepad2.dpad_down; // dpad-down
        boolean clawDown = gamepad2.a || clawDownSafe; // A

        // Robot Lift
        boolean robotLiftRotation = gamepad2.right_trigger > 0.05; // RT
        boolean robotLiftExtend = gamepad2.right_trigger > 0.5; // RT
        boolean robotLiftReset = gamepad2.right_stick_button;

        // Gantry
        boolean screwArmToggle = gamepad2.x; // X
        boolean screwDeposit = gamepad2.left_trigger > 0.25; // LT
        boolean screwIntake = gamepad2.left_bumper; // LB
        boolean slideUp = gamepad2.left_stick_y < -0.25; // Left Y Axis Joystick
        boolean slideDown = gamepad2.left_stick_y > 0.25; // Left Y Axis Joystick
        double gantryXInput = -gamepad2.right_stick_x;

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
            if (!clawDownSafe) {
                this.robot.getClaw().open();
            }
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

        if (slideUp) {
            this.robot.getGantry().setSlideTarget(SLIDE_UP);
        } else if (slideDown) {
            this.robot.getGantry().setSlideTarget(0);
        } else if (previousSlideUp || previousSlideDown){
            this.robot.getGantry().setSlideTarget(this.robot.getGantry().getSlidePosition());
        }

        double gantryXPosition = X_CENTER + (gantryXInput * 0.5);
        gantryXPosition = Math.max(X_MIN, Math.min(gantryXPosition, X_MAX));
        this.robot.getGantry().setX(gantryXPosition);

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


        this.previousSlideUp = slideUp;
        this.previousScrewArmToggle = screwArmToggle;
        this.previousRobotLiftReset = robotLiftReset;
        this.previousRobotLiftExtend = robotLiftExtend;

        this.robot.update();
    }
}
