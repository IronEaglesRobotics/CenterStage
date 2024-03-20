package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Slides {
    public DcMotor slide;
    public DcMotor slide2;

//    public static double p = 0.0014;
//    public static double i = 0.02;
//    public static double d = 0;
//    public static double f = 0.01;
    //p was 0.0014
    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.0015,0.04,0,0.01);
    public static double pTolerance = 20;
    private PIDController controller = new PIDController(coefficients.p, coefficients.i, coefficients.d);

    public static int targetMin = -10;
    public static int targetMax = 830;

    public static int down = 0;

    public static int micro_tier1 = 50;
    public static int mini_tier1 = 250;
    public static int tier1 = 350;
    public static int tier2 = 500;
    public static int tier3 = 720;

    private int target = 0;

    public static int manualSpeed = 50;

    public enum Position { DOWN, mini_tier1,  TIER1, TIER2, TIER3 }

    public Slides(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotor.class, "Rightslide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide2 = hardwareMap.get(DcMotor.class, "Leftslide");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTarget(int pos) {
        target = Math.min(Math.max(pos, targetMin), targetMax);
    }

    public void setTarget(Position pos) {
        if (pos == Position.DOWN) {
            target = Math.min(Math.max(down, targetMin), targetMax);
        } else if (pos == Position.mini_tier1) {
            target = Math.min(Math.max(mini_tier1, targetMin), targetMax);
        } else if (pos == Position.TIER1) {
            target = Math.min(Math.max(tier1, targetMin), targetMax);
        } else if (pos == Position.TIER2) {
            target = Math.min(Math.max(tier2, targetMin), targetMax);
        } else if (pos == Position.TIER3) {
            target = Math.min(Math.max(tier3, targetMin), targetMax);
        }
    }

    public void increaseTarget(double increase) {
        target += (int) (increase * manualSpeed);
        target = Math.min(targetMax, Math.max(targetMin, target));
    }

    public int getTarget() {
        return target;
    }



    public boolean atTarget() {
        return controller.atSetPoint();
    }

    public void cancel() {
        target = slide.getCurrentPosition();
    }

    public void targetReset() {
        target = targetMin;
    }

    public void update(double runTime) {
//        highPos = 720 + heightOffset;
//        midPos = 350 + heightOffset;
//        lowPos = heightOffset;
//        pickupPos = 20 + heightOffset;
//        downPos = heightOffset;// TODO add these back in

//        if (slide.getCurrentPosition() <= pTolerance && target <= pTolerance) {
//            slide.setPower(0);
//            slide2.setPower(0);
//        } else {

            double pid, ff;
            controller.setPID(coefficients.p, coefficients.i, coefficients.d);
            controller.setTolerance(pTolerance);

            pid = controller.calculate(slide.getCurrentPosition(), target);
            ff = coefficients.f;
            slide.setPower(pid + ff);

            pid = controller.calculate(slide2.getCurrentPosition(), target);
            ff = coefficients.f;
            slide2.setPower(pid + ff);
//            }
//        }
        }


    public String getTelemetry() {
        return String.format("Position: %s %s\nTarget: %s %s\nPower: %s %s", slide.getCurrentPosition(), slide2.getCurrentPosition(), target, target, slide.getPower(), slide2.getPower());
    }
}