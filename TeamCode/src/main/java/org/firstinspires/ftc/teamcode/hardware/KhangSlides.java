package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class KhangSlides {

    //Create and assign motors
    public KhangSlides(HardwareMap hardwareMap) {
        rSMotor = hardwareMap.get(DcMotor.class, "Right Slide Motor");
//        rSMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSMotor = hardwareMap.get(DcMotor.class, "Left Slide Motor");
        lSMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //make each motor
    private DcMotor rSMotor;
    private DcMotor lSMotor;

    //Value for motor power and speed slides go at
    public static double rSPower = 0.5;
    public static double lSPower = 0.5;
    public static double downpos = 0;
    public static double tape1pos = 200;
    public static double tape2pos = 350;
    public static double tape3pos = 500;
    public static double maxpos = 1000;

    //Set default slide height to 0 aka starting position
    public static double sHeight = 0;
    public static double newSHeight = 0;

//    Do stuff to get to a position when asked to go to a position
    public void setSPos(SlidePos.SPosition height) {
        if (height == SlidePos.SPosition.DOWN) {
            sHeight = 0;
            lSMotor.setTargetPosition((int) downpos);
            lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSMotor.setPower(lSPower);
            rSMotor.setTargetPosition((int) downpos);
            rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rSMotor.setPower(rSPower);
        } else if (height == SlidePos.SPosition.TAPE1) {
            sHeight = 300;
            lSMotor.setTargetPosition((int) tape1pos);
            lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSMotor.setPower(lSPower);
            rSMotor.setTargetPosition((int) tape1pos);
            rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rSMotor.setPower(rSPower);
        } else if (height == SlidePos.SPosition.TAPE2) {
            sHeight = 500;
            lSMotor.setTargetPosition((int) tape2pos);
            lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSMotor.setPower(lSPower);
            rSMotor.setTargetPosition((int) tape2pos);
            rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rSMotor.setPower(rSPower);
        } else if (height == SlidePos.SPosition.TAPE3) {
            sHeight = 700;
            lSMotor.setTargetPosition((int) tape3pos);
            lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSMotor.setPower(lSPower);
            rSMotor.setTargetPosition((int) tape3pos);
            rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rSMotor.setPower(rSPower);
        } else if (height == SlidePos.SPosition.MAX) {
            sHeight = 1000;
            lSMotor.setTargetPosition((int) maxpos);
            lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSMotor.setPower(lSPower);
            rSMotor.setTargetPosition((int) maxpos);
            rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rSMotor.setPower(rSPower);
        }
    }

//    public void setHeight(Hieght.height send) {
//        if (send == Hieght.height.ASCEND) {
//            if (newSHeight <= 790) {
//                newSHeight = newSHeight + 10;
//                lSMotor.setTargetPosition((int) (newSHeight));
//                lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lSMotor.setPower(lSPower);
//                rSMotor.setTargetPosition((int) (newSHeight));
//                rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rSMotor.setPower(rSPower);
//            } else if (newSHeight >= 800) {
//                newSHeight = 800;
//                lSMotor.setTargetPosition((int) (newSHeight));
//                lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lSMotor.setPower(lSPower);
//                rSMotor.setTargetPosition((int) (newSHeight));
//                rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rSMotor.setPower(rSPower);
//            }
//            } else if (send == Hieght.height.HOLD) {
//                newSHeight = newSHeight + 0;
//                lSMotor.setTargetPosition((int) (newSHeight));
//                lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lSMotor.setPower(lSPower);
//                rSMotor.setTargetPosition((int) (newSHeight));
//                rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rSMotor.setPower(rSPower);
//            } else if (send == Hieght.height.DESCEND) {
//                if (newSHeight >= 10) {
//                    newSHeight = newSHeight - 10;
//                    lSMotor.setTargetPosition((int) (newSHeight));
//                    lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    lSMotor.setPower(lSPower);
//                    rSMotor.setTargetPosition((int) (newSHeight));
//                    rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rSMotor.setPower(rSPower);
//                } else if (newSHeight <= 0) {
//                    newSHeight = 0;
//                    lSMotor.setTargetPosition((int) (newSHeight));
//                    lSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    lSMotor.setPower(lSPower);
//                    rSMotor.setTargetPosition((int) (newSHeight));
//                    rSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rSMotor.setPower(rSPower);
//                }
//            }
//        }


    //Return feed to the control hub for easy debug
    public String getTelemetry() {
        return "Left Slide Motor: "+lSMotor.getPower()+"Right Slide Motor: "+rSMotor.getPower();
    }
}
