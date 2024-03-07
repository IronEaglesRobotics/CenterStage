package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Configurables.ARMACCSCORE;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARMPICKUPSTACK;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARMREST;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARMSCORE;
import static org.firstinspires.ftc.teamcode.util.Configurables.BIGOPEN;
import static org.firstinspires.ftc.teamcode.util.Configurables.CLOSE;
import static org.firstinspires.ftc.teamcode.util.Configurables.HANG;
import static org.firstinspires.ftc.teamcode.util.Configurables.HANGPLANE;
import static org.firstinspires.ftc.teamcode.util.Configurables.HANGRELEASE;
import static org.firstinspires.ftc.teamcode.util.Configurables.OPEN;
import static org.firstinspires.ftc.teamcode.util.Configurables.PICKUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.PLANELAUNCH;
import static org.firstinspires.ftc.teamcode.util.Configurables.PLANELOCK;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDEAUTOSTACKS;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDELAYERONE;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDELAYERTWO;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDEPICKUPSTACKSTWO;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDEUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_POWER_DOWN;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_POWER_UP;
import static org.firstinspires.ftc.teamcode.util.Configurables.WRISTPICKUP;
import static org.firstinspires.ftc.teamcode.util.Configurables.WRISTSCORE;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW;
import static org.firstinspires.ftc.teamcode.util.Constants.HANGLEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.HANGRIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFTARM;
import static org.firstinspires.ftc.teamcode.util.Constants.LIGHTS;
import static org.firstinspires.ftc.teamcode.util.Constants.PLANE;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHTARM;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDELEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDERIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WRIST;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.Camera;

import lombok.Getter;
@Config
public class Robot {
    @Getter
    private MecanumDrive drive;
    @Getter
    private Led led;
    @Getter
    private Arm arm;
    @Getter
    private Wrist wrist;
    @Getter
    private Claw claw;
    @Getter
    private Hang hang;
    @Getter
    private Camera camera;
    @Getter
    private Plane plane;
    @Getter
    private Slides slides;



    public Robot init(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap);
        this.hang = new Hang().init(hardwareMap);
        this.arm = new Arm().init(hardwareMap);
        this.wrist = new Wrist().init(hardwareMap);
        this.claw = new Claw().init(hardwareMap);
//        this.camera = new Camera(hardwareMap);
        this.plane = new Plane().init(hardwareMap);
        this.slides= new Slides().init(hardwareMap);
//        this.led = new Led().init(hardwareMap);
        return this;
    }

    public static class Slides {
        private DcMotor slidesRight = null;
        public DcMotor slidesLeft = null;

        public Slides init(HardwareMap hardwareMap) {
            this.slidesLeft = hardwareMap.get(DcMotor.class, SLIDELEFT);
            this.slidesRight = hardwareMap.get(DcMotor.class, SLIDERIGHT);
            this.slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slidesRight.setTargetPosition(0);
            this.slidesLeft.setTargetPosition(0);

            this.slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return this;
        }

        public void slideTo(int position, double power) {
            this.slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesLeft.setTargetPosition(position);
            this.slidesLeft.setPower(power);

            this.slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesRight.setTargetPosition(position);
            this.slidesRight.setPower(power);
        }

        public void slideUp(){this.slideTo(SLIDEUP, SLIDE_POWER_UP);}

        public void slideDown(){this.slideTo(0, SLIDE_POWER_DOWN);}

        public void slideFirstLayer(){this.slideTo(SLIDELAYERONE, SLIDE_POWER_UP);}

        public void slideScoreStack(){this.slideTo(SLIDELAYERTWO, SLIDE_POWER_UP);}

        public void slideAutoStacks(){this.slideTo(SLIDEAUTOSTACKS, SLIDE_POWER_UP);}

        public void slidePickupStackTwo(){this.slideTo(SLIDEPICKUPSTACKSTWO, SLIDE_POWER_UP);}

        public void slideStop() {
            this.slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesLeft.setTargetPosition(this.slidesLeft.getCurrentPosition());
            this.slidesLeft.setPower(1);

            this.slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesRight.setTargetPosition(this.slidesRight.getCurrentPosition());
            this.slidesRight.setPower(1);}

    }

    public static class Hang {
        public DcMotor hangLeft = null;
        public DcMotor hangRight = null;

        public Hang init(HardwareMap hardwareMap) {
            this.hangLeft = hardwareMap.get(DcMotor.class, HANGLEFT);
            this.hangRight = hardwareMap.get(DcMotor.class, HANGRIGHT);
            this.hangLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hangRight.setTargetPosition(0);
            this.hangLeft.setTargetPosition(0);

            this.hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return this;
        }

        public void hangTo(int hangPos, double daPower) {
            this.hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hangLeft.setTargetPosition(hangPos);
            this.hangLeft.setPower(daPower);

            this.hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hangRight.setTargetPosition(hangPos);
            this.hangRight.setPower(daPower);
        }

        public void hangRelease(){
            this.hangTo(HANGRELEASE,1);
        }

        public void hang(){
            this.hangTo(HANG,1);
        }
        public void hangPlane(){
            this.hangTo(HANGPLANE,1);
        }

        public void hangIdle() {
            this.hangLeft.setPower(0);
            this.hangRight.setPower(0);
        }



    }

    public static class Arm {
        private Servo leftArm;
        private Servo rightArm;


        public Arm init(HardwareMap hardwareMap) {
            this.leftArm = hardwareMap.get(Servo.class, LEFTARM);
            this.rightArm = hardwareMap.get(Servo.class, RIGHTARM);
            this.rightArm.setDirection(Servo.Direction.REVERSE);
            this.rightArm.setPosition(ARMREST);
            this.leftArm.setPosition(ARMREST);
            return this;
        }

        public void pickup() {
            this.leftArm.setPosition(PICKUP);
            this.rightArm.setPosition(PICKUP);
        }

        public void armScore() {
            this.leftArm.setPosition(ARMSCORE);
            this.rightArm.setPosition(ARMSCORE);
        }

        public void armSecondaryScore() {
            this.leftArm.setPosition(ARMACCSCORE);
            this.rightArm.setPosition(ARMACCSCORE);
        }


        public void armPickupStack() {
            this.leftArm.setPosition(ARMPICKUPSTACK);
            this.rightArm.setPosition(ARMPICKUPSTACK);
        }

        public void armRest() {
            this.leftArm.setPosition(ARMREST);
            this.rightArm.setPosition(ARMREST);
        }
    }

    public static class Wrist {
        private Servo wrist;

        public Wrist init(HardwareMap hardwareMap) {
            this.wrist = hardwareMap.get(Servo.class, WRIST);
            this.wrist.setPosition(WRISTPICKUP);
            return this;
        }

        public void wristPickup() {
            this.wrist.setPosition(WRISTPICKUP);
        }

        public void wristScore() {
            this.wrist.setPosition(WRISTSCORE);
        }

    }

    public static boolean clawIsOpen;

    public static class Claw {
        private Servo claw;

        public Claw init(HardwareMap hardwareMap) {
            this.claw = hardwareMap.get(Servo.class, CLAW);
            close();
            return this;
        }

        public void close() {
            this.claw.setPosition(CLOSE);
            clawIsOpen = false;
        }

        public void open() {
            this.claw.setPosition(OPEN);
            clawIsOpen = true;
        }

        public void openStack() {
            this.claw.setPosition(BIGOPEN);
            clawIsOpen = true;
        }

        public void setPos(double pos) {
            this.claw.setPosition(pos);
        }

    }

    public static class Led {
        private RevBlinkinLedDriver led;

        public Led init(HardwareMap hardwareMap) {
            this.led = hardwareMap.get(RevBlinkinLedDriver.class, LIGHTS);
            return this;
        }

        public void LED() {
            if (clawIsOpen) {
                white();
            } else {
                gold();
            }
        }

        public void gold() {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }

        public void white() {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        }
    }

    public static class Plane {
        private Servo plane;

        public Plane init(HardwareMap hardwareMap) {
            this.plane = hardwareMap.get(Servo.class, PLANE);
            this.plane.setPosition(PLANELOCK);
            return this;
        }

        public void planeLock() {
            this.plane.setPosition(PLANELOCK);
        }

        public void planeLaunch() {
            this.plane.setPosition(PLANELAUNCH);
        }

    }

    public void pickupMacro(GamepadEx gamepadEx, double runtime) {
        switch (pickupMacroState) {
            case IDLE:
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    pickupMacroState = pickupMacroStates.OPEN;
                }
                break;
            case OPEN:
                    delay = runtime + .2;
                    this.getClaw().open();
                    pickupMacroState = pickupMacroStates.DROP;
                break;
            case DROP:
                    if (runtime > delay) {
                        this.getArm().pickup();
                        delay= runtime + .2;
                        pickupMacroState = pickupMacroStates.CLOSE;
                    }
                break;
            case CLOSE:
                if (runtime > delay) {
                    this.getClaw().close();
                    delay= runtime + .1;
                    pickupMacroState = pickupMacroStates.NEUTRAL;
                }
                break;
            case NEUTRAL:
                if (runtime > delay) {
                    this.getArm().armRest();
                    pickupMacroState = pickupMacroStates.IDLE;
                }
                break;

        }
    }

    public pickupMacroStates pickupMacroState = pickupMacroStates.IDLE;

    public enum pickupMacroStates{
                IDLE,
                OPEN,
                DROP,
                CLOSE,
                NEUTRAL

        }

    double delay;

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        this.drive.update();
        return this.drive.trajectorySequenceBuilder(this.drive.getPoseEstimate());
    }
}

