package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_DELTA_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.X_CENTER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_SCREW_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_X_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LEFT_SLIDE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.RIGHT_SLIDE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.X_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.X_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.X_MIN;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry {
    private final Servo xServo;
    private final Servo armServo;
    private final CRServo screwServo;
    private final DcMotor liftLeft;
    private final DcMotor liftRight;
    PController armController = new PController(GANTRY_ARM_KP);
    private double armControllerTarget;
    PController xController = new PController(X_KP);
    private double xControllerTarget;
    private Telemetry telemetry;

    public Gantry(HardwareMap hardwareMap) {
        this.xServo = hardwareMap.get(Servo.class, GANTRY_X_NAME);
        this.armServo = hardwareMap.get(Servo.class, GANTRY_ARM_NAME);
        this.screwServo = hardwareMap.get(CRServo.class, GANTRY_SCREW_NAME);
        this.armServo.setPosition(GANTRY_ARM_MIN);

        this.liftLeft = hardwareMap.get(DcMotor.class, LEFT_SLIDE_MOTOR_NAME);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftLeft.setTargetPosition(0);
        this.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.liftRight = hardwareMap.get(DcMotor.class, RIGHT_SLIDE_MOTOR_NAME);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftRight.setTargetPosition(0);
        this.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.xControllerTarget = X_MIN;
    }

    public Gantry(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public void setSlideTarget(int target) {
        this.liftLeft.setTargetPosition(target);
        this.liftLeft.setPower(SLIDE_POWER);

        this.liftRight.setTargetPosition(target);
        this.liftRight.setPower(SLIDE_POWER);
    }

    public void setX(double x)
    {
        this.xControllerTarget = Math.max(X_MIN, Math.min(x, X_MAX));
    }

    public double getX() {
        return this.xServo.getPosition();
    }

    public void armOut() {
        this.armControllerTarget = GANTRY_ARM_MAX;
    }

    public void armIn() {
        this.armControllerTarget = GANTRY_ARM_MIN;
    }

    public void intake() {
        this.screwServo.setPower(1);
        this.screwServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void deposit() {
        this.screwServo.setPower(1);
        this.screwServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
        this.screwServo.setPower(0);
    }

    public void center() {
        this.setX(X_CENTER);
    }

    public int getSlidePosition() {
        return this.liftLeft.getCurrentPosition();
    }

    public boolean isIn() {
        double fudge = (GANTRY_ARM_MAX - GANTRY_ARM_MIN) * .75;
        return this.armServo.getPosition() < GANTRY_ARM_MIN + fudge;
    }

    public void update() {
        this.armController.setSetPoint(this.armControllerTarget);
        this.armController.setTolerance(0.001);
        this.armController.setP(GANTRY_ARM_KP);

        this.xController.setSetPoint(this.xControllerTarget);
        this.xController.setTolerance(0.001);
        this.xController.setP(X_KP);

        double armOutput = 0;
        if (!this.armController.atSetPoint()) {
            armOutput = Math.max(-1 * GANTRY_ARM_DELTA_MAX, Math.min(GANTRY_ARM_DELTA_MAX, this.armController.calculate(armServo.getPosition())));
            this.armServo.setPosition(this.armServo.getPosition() + armOutput);
        }

        double xOutput = 0;
        if (!this.xController.atSetPoint()) {
            xOutput = this.xController.calculate(this.xServo.getPosition());
            this.xServo.setPosition(this.xServo.getPosition() + xOutput);
        }
    }
}
