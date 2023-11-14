package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_ARM_KP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_EXTEND_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_RETRACT_PCT;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_ROTATION_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_ROTATION_UP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.ROBOT_LIFT_LIFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.ROBOT_LIFT_ROTATION_NAME;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotLift implements Updatable{
    private Servo rotation;
    private DcMotor lift;
    PController armController = new PController(LIFT_ARM_KP);
    private double armControllerTarget;
    private Telemetry telemetry;

    public RobotLift(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }

    public RobotLift(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        this.rotation = hardwareMap.get(Servo.class, ROBOT_LIFT_ROTATION_NAME);

        this.lift = hardwareMap.get(DcMotor.class, ROBOT_LIFT_LIFT_NAME);
        this.lift.setTargetPosition(0);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up() {
        this.armControllerTarget = LIFT_ROTATION_UP;
    }

    public void extend() {
        this.armControllerTarget = LIFT_ROTATION_UP;
        this.lift.setTargetPosition(LIFT_EXTEND_MAX);

        this.lift.setPower(1);
    }

    public void retract() {
        this.armControllerTarget = LIFT_ROTATION_UP;
        int liftTarget = (int)(LIFT_EXTEND_MAX * (1 - LIFT_RETRACT_PCT));
        int target = this.lift.getCurrentPosition() < liftTarget ? 0 : liftTarget;
        this.lift.setTargetPosition(target);

        this.lift.setPower(1);
    }

    public void startReset() {
        this.armControllerTarget = LIFT_ROTATION_DOWN;
        this.lift.setTargetPosition(-1 * LIFT_EXTEND_MAX);
        this.lift.setPower(1);
    }

    public void stopReset() {
        this.armControllerTarget = LIFT_ROTATION_DOWN;
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(0);
    }

    public void update() {
        this.armController.setP(LIFT_ARM_KP);
        this.armController.setSetPoint(this.armControllerTarget);
        double output = this.armController.calculate(this.rotation.getPosition());
        this.rotation.setPosition(this.rotation.getPosition() + output);
    }

    public boolean isUp() {
        return this.armControllerTarget == LIFT_ROTATION_UP;
    }
}