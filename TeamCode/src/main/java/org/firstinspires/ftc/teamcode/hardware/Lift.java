package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LIFT_UP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lombok.Getter;

public class Lift {
    protected final DcMotor lift;

    protected Lift(HardwareMap hardwareMap) {
        this.lift = hardwareMap.get(DcMotor.class, LIFT_MOTOR_NAME);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void up() {
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setTargetPosition(LIFT_UP);
    }

    public void down() {
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setTargetPosition(0);
    }

    public void setInput(double x) {
        this.lift.setPower(x);
    }
}
