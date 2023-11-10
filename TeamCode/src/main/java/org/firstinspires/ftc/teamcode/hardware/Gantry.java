package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_DELTA_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_MIN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_ARM_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_CENTER;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_SCREW_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.GANTRY_X_NAME;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry extends Slide {
    private final Servo xServo;
    private final Servo armServo;
    private final CRServo screwServo;
    PController armController = new PController(GANTRY_ARM_KP);
    private double armControllerTarget;

    private Telemetry telemetry;

    public Gantry(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.xServo = hardwareMap.get(Servo.class, GANTRY_X_NAME);
        this.armServo = hardwareMap.get(Servo.class, GANTRY_ARM_NAME);
        this.screwServo = hardwareMap.get(CRServo.class, GANTRY_SCREW_NAME);
        this.armServo.setPosition(GANTRY_ARM_MIN);
    }

    public Gantry(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public void setX(double x) {
        this.xServo.setPosition(x);
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
        this.setX(GANTRY_CENTER);
    }

    public boolean isIn() {
        double fudge = (GANTRY_ARM_MAX - GANTRY_ARM_MIN) * .75;
        return this.armServo.getPosition() < GANTRY_ARM_MIN + fudge;
    }

    public void update() {
        this.armController.setSetPoint(this.armControllerTarget);
        this.armController.setTolerance(0.001);
        this.armController.setP(GANTRY_ARM_KP);

        double output = 0;
        if (!this.armController.atSetPoint()) {
            output = Math.max(-1 * GANTRY_ARM_DELTA_MAX, Math.min(GANTRY_ARM_DELTA_MAX, this.armController.calculate(armServo.getPosition())));
            this.armServo.setPosition(this.armServo.getPosition() + output);
        }

        this.telemetry.addData("Arm P Controller", output);
        this.telemetry.addData("Arm P Setpoint", this.armControllerTarget);
        this.telemetry.addData("Arm In", this.isIn());
    }
}
