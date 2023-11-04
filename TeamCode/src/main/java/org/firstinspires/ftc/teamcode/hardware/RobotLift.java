package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_EXTEND_MAX;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_ROTATION_UP;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.ROBOT_LIFT_LIFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.ROBOT_LIFT_ROTATION_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotLift {
    private DcMotor rotation;
    private DcMotor lift;

    public RobotLift(HardwareMap hardwareMap) {
        this.init(hardwareMap);
    }
    public void init(HardwareMap hardwareMap) {
        this.rotation = hardwareMap.get(DcMotor.class, ROBOT_LIFT_ROTATION_NAME);
        this.rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.lift = hardwareMap.get(DcMotor.class, ROBOT_LIFT_LIFT_NAME);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void raise() {
        this.rotation.setTargetPosition(LIFT_ROTATION_UP);
        this.lift.setTargetPosition(LIFT_EXTEND_MAX);

        this.rotation.setPower(1);
        this.lift.setPower(1);
    }

    public void lift() {
        this.rotation.setTargetPosition(LIFT_ROTATION_UP);
        this.lift.setTargetPosition(0);

        this.lift.setPower(1);
        this.rotation.setPower(1);
    }

    public void reset() {
        this.rotation.setTargetPosition(0);
        this.lift.setTargetPosition(0);

        this.lift.setPower(0.25);
        this.rotation.setPower(0.25);
    }
}