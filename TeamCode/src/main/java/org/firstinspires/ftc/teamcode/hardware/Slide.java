package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.SLIDE_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    protected final DcMotor lift;

    protected Slide(HardwareMap hardwareMap) {
        this.lift = hardwareMap.get(DcMotor.class, SLIDE_MOTOR_NAME);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTarget(int target) {
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setTargetPosition(target);
        this.lift.setPower(1);
    }

    public void setInput(double x) {
        this.lift.setPower(x);
    }

    public int getSlidePosition() {
        return this.lift.getCurrentPosition();
    }
}
