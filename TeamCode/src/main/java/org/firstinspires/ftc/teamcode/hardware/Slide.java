package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.LEFT_SLIDE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.RIGHT_SLIDE_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    protected final DcMotor left;
    protected final DcMotor right;

    protected Slide(HardwareMap hardwareMap) {
        this.left = hardwareMap.get(DcMotor.class, LEFT_SLIDE_MOTOR_NAME);
        this.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.right = hardwareMap.get(DcMotor.class, RIGHT_SLIDE_MOTOR_NAME);
        this.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTarget(int target) {
        this.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.left.setTargetPosition(target);
        this.left.setPower(1);

        this.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right.setTargetPosition(target);
        this.right.setPower(1);
    }

    public void setInput(double x) {

    }

    public int getSlidePosition() {
        return this.left.getCurrentPosition();
    }
}
