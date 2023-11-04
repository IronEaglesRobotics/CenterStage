package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.BACK_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.BACK_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.FRONT_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.FRONT_RIGHT_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

public class Drive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public Drive(HardwareMap hardwareMap) {
        // Drive
        this.frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        this.frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        this.backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        this.backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);
        this.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.frontRight.setDirection(DcMotor.Direction.FORWARD);
        this.backLeft.setDirection(DcMotor.Direction.REVERSE);
        this.backRight.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setInput(double x, double y, double z) {
        // instantiate motor power variables
        double flPower, frPower, blPower, brPower;

        flPower = x + y + z;
        frPower = -x + y - z;
        blPower = -x + y + z;
        brPower = x + y - z;

        double max = Math.max(Math.max(flPower, frPower), Math.max(blPower, brPower));
        if (max > 1) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // actually set the motor powers
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public String getTelemetry() {
        double flPower = this.frontLeft.getPower();
        double frPower = this.frontRight.getPower();
        double blPower = this.backLeft.getPower();
        double brPower = this.backRight.getPower();

        return String.format("FL: %f, FR: %f, BL: %f, BR: %f", flPower, frPower, blPower, brPower);
    }

    public void setInput(Gamepad gamepad1, Gamepad gamepad2) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        setInput(x, y, z);
    }
}