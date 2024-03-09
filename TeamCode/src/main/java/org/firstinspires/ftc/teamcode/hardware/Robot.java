package org.firstinspires.ftc.teamcode.hardware;

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
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    public static boolean clawIsOpen;
    public static double WRISTDELAY = .08;
    double delay;
    public pickupMacroStates pickupMacroState = pickupMacroStates.IDLE;
    public armMacroStates armMacroState = armMacroStates.IDLE;
    @Getter
    public Arm arm;
    @Getter
    private MecanumDrive drive;
    @Getter
    private Led led;
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
        this.slides = new Slides().init(hardwareMap);
//        this.led = new Led().init(hardwareMap);
        return this;
    }

    public void pickupMacro(GamepadEx gamepadEx, double runtime) {
        switch (pickupMacroState) {
            case IDLE:
                if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    pickupMacroState = pickupMacroStates.DROP;
                }
                break;
            case OPEN:
                delay = runtime + .3;
                this.getClaw().open();
                pickupMacroState = pickupMacroStates.DROP;
                break;
            case DROP:
                if (runtime > delay) {
                    this.getArm().pickup(true);
                    delay = runtime + .2;
                    pickupMacroState = pickupMacroStates.CLOSE;
                }
                break;
            case CLOSE:
                if (runtime > delay) {
                    this.getClaw().close();
                    delay = runtime + .3;
                    pickupMacroState = pickupMacroStates.NEUTRAL;
                }
                break;
            case NEUTRAL:
                if (runtime > delay) {
                    this.getArm().armRest(true);
                    pickupMacroState = pickupMacroStates.IDLE;
                }
                break;

        }
    }

    public void armMacro(GamepadEx gamepadEx, double runtime) {
        switch (armMacroState) {
            case IDLE:
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
                    armMacroState = armMacroStates.ARMSWING;
                }
                break;
            case ARMSWING:
                arm.armSecondaryScore();
                delay = runtime + WRISTDELAY;
                armMacroState = armMacroStates.WRIST;
                break;
            case WRIST:
                if (runtime > delay) {
                    wrist.wristScore();
                    armMacroState = armMacroStates.IDLE;
                }
                break;
        }
    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        this.drive.update();
        return this.drive.trajectorySequenceBuilder(this.drive.getPoseEstimate());
    }

    public void update() {
        this.arm.update();
        this.wrist.update();
        this.drive.update();
    }

    public enum pickupMacroStates {
        IDLE, OPEN, DROP, CLOSE, NEUTRAL

    }

    public enum armMacroStates {
        IDLE, ARMSWING, WRIST
    }

    @Config
    public static class Slides {
        //Values
        public static double SLIDE_POWER_UP = .7;
        public static double SLIDE_POWER_DOWN = .2;
        public static int SLIDELAYERONE = 60;
        public static int SLIDEAUTOSTACKS = 250;
        public static int SLIDEUP = 630;
        public static int SLIDELAYERTWO = 350;
        public static int SLIDESTACK = 80;
        public static int SLIDEPICKUPSTACKSTWO = 30;
        //Motors
        public DcMotor slidesRight = null;
        public DcMotor slidesLeft = null;

        public Slides init(HardwareMap hardwareMap) {
            this.slidesLeft = hardwareMap.get(DcMotor.class, SLIDELEFT);
            this.slidesRight = hardwareMap.get(DcMotor.class, SLIDERIGHT);
            this.slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slidesLeft.setTargetPosition(0);
            this.slidesRight.setTargetPosition(0);

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

        public void slideUp() {
            this.slideTo(SLIDEUP, SLIDE_POWER_UP);
        }

        public void slideDown() {
            this.slideTo(0, SLIDE_POWER_DOWN);
        }

        public void slideFirstLayer() {
            this.slideTo(SLIDELAYERONE, SLIDE_POWER_UP);
        }

        public void slideScoreStack() {
            this.slideTo(SLIDELAYERTWO, SLIDE_POWER_UP);
        }

        public void slideAutoStacks() {
            this.slideTo(SLIDEAUTOSTACKS, SLIDE_POWER_UP);
        }

        public void slidePickupStackTwo() {
            this.slideTo(SLIDEPICKUPSTACKSTWO, SLIDE_POWER_UP);
        }

        public void slideStop() {
            this.slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesLeft.setTargetPosition(this.slidesLeft.getCurrentPosition());
            this.slidesLeft.setPower(1);

            this.slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.slidesRight.setTargetPosition(this.slidesRight.getCurrentPosition());
            this.slidesRight.setPower(1);
        }

    }

    @Config
    public static class Hang {
        //Values
        public static int HANGRELEASE = 1550;
        public static int HANG = 0;
        public static int HANGPLANE = 1150;
        //Motors
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

        public void hangTo(int hangPos, double Power) {
            this.hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hangLeft.setTargetPosition(hangPos);
            this.hangLeft.setPower(Power);

            this.hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hangRight.setTargetPosition(hangPos);
            this.hangRight.setPower(Power);
        }

        public void hangRelease() {
            this.hangTo(HANGRELEASE, 1);
        }

        public void hang() {
            this.hangTo(HANG, 1);
        }

        public void hangPlane() {
            this.hangTo(HANGPLANE, 1);
        }
    }

    @Config
    public static class Arm {
        //Values
        public static double DIFFY_KP = 0.1;
        public static double DIFFY_TOL = 0.01;
        public static double ARM_MAX_DELTA = 0.02;
        public static double ARMREST = 0.89;
        public static double ARMSCORE = 0.4;
        public static double ARMACCSCORE = .57;
        public static double ARMPICKUPSTACK = 0.815;
        public static double PICKUP = 0.92;
        //PControler
        public PController armPController;
        private double armTarget;
        //Servos
        private Servo leftArm;
        private Servo rightArm;

        public Arm init(HardwareMap hardwareMap) {
            this.leftArm = hardwareMap.get(Servo.class, LEFTARM);
            this.rightArm = hardwareMap.get(Servo.class, RIGHTARM);
            this.rightArm.setDirection(Servo.Direction.REVERSE);
            this.rightArm.setPosition(ARMREST);
            this.leftArm.setPosition(ARMREST);
            this.armPController = new PController(DIFFY_KP);
            this.armRest(true);
            return this;
        }

        public void pickup() {
            pickup(false);
        }

        public void pickup(boolean now) {
            moveArm(PICKUP, now);
        }

        public void armScore() {
            this.armSecondaryScore();
        }

        public void armSecondaryScore() {
            moveArm(ARMACCSCORE, false);
        }

        public void armPickupStack() {
            moveArm(ARMPICKUPSTACK, false);
        }

        public void armRest() {
            armRest(false);
        }

        public void armRest(boolean now) {
            moveArm(ARMREST, now);
        }

        private void moveArm(double position, boolean now) {
            this.armTarget = position;
            this.armPController.setSetPoint(this.armTarget);
            if (now) {
                this.leftArm.setPosition(position);
                this.rightArm.setPosition(position);
            }
        }

        public boolean isAtTarget() {
            return this.armPController.atSetPoint();
        }

        public void update() {
            this.armPController.setSetPoint(this.armTarget);
            this.armPController.setTolerance(DIFFY_TOL);
            this.armPController.setP(DIFFY_KP);

            if (!isAtTarget()) {
                double armDelta = this.armPController.calculate(leftArm.getPosition());
                if (Math.abs(this.armPController.getPositionError()) > 0.1) {
                    armDelta = Math.copySign(ARM_MAX_DELTA, armDelta);
                }
                this.leftArm.setPosition(leftArm.getPosition() + armDelta);
                this.rightArm.setPosition(rightArm.getPosition() + armDelta);
            }

//
//            this.rightPController.setSetPoint(this.rightTarget);
//            this.rightPController.setTolerance(DIFFY_TOL);
//            this.rightPController.setP(DIFFY_KP);
//            double rightDelta = this.rightPController.calculate(rightArm.getPosition());
//            rightDelta = Math.max(-1 * DIFFY_MAX_DELTA, Math.min(DIFFY_MAX_DELTA, rightDelta));
//            this.rightArm.setPosition(rightArm.getPosition() + rightDelta);

            // A is where I am
            // B is where I want to get to
            // E is the difference between them
            // X is the midpoint between A and B
            // while I am at on the left side of X, delta should be constant
            // once I am on the right side of X, delta should be whatever ther P controller says


        }

    }

    @Config
    public static class Wrist {
        //Pcontroller
        public static double KP = 0.2;
        public static double TOL = 0.005;
        public static double MAX_DELTA = 0.04;
        private PIDFController wristPController;
        //Values
        public static double WRISTPICKUP = 0.3;
        public static double WRISTSCORE = .98;
        //Servo
        private Servo wrist;

        public Wrist init(HardwareMap hardwareMap) {
            this.wrist = hardwareMap.get(Servo.class, WRIST);
            this.wrist.setPosition(WRISTPICKUP);
            this.wristPController = new PController(KP);
            this.wristPController.setSetPoint(WRISTPICKUP);
            return this;
        }

        public void wristPickup() {
            this.wristPController.setSetPoint(WRISTPICKUP);
        }

        public void wristScore() {
            this.wristPController.setSetPoint(WRISTSCORE);
        }

        public void update() {
            this.wristPController.setTolerance(TOL);
            this.wristPController.setP(KP);

            if (!wristPController.atSetPoint()) {
                double wristDelta = this.wristPController.calculate(wrist.getPosition());
                if (Math.abs(this.wristPController.getPositionError()) > 0.1) {
                    wristDelta = Math.copySign(MAX_DELTA, wristDelta);
                }
                this.wrist.setPosition(wrist.getPosition() + wristDelta);
            }
        }
    }

    @Config
    public static class Claw {
        //Values
        public static double OPEN = 0.65;
        public static double BIGOPEN = 0f;
        public static double CLOSE = 0.73;
        public static double CLAW_MIN = 0;
        public static double CLAW_MAX = 1;
        //Servo
        private Servo claw;

        public Claw init(HardwareMap hardwareMap) {
            this.claw = hardwareMap.get(Servo.class, CLAW);
            this.claw.scaleRange(CLAW_MIN, CLAW_MAX);
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

    @Config
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

    @Config
    public static class Plane {
        //Values
        public static double PLANELOCK = 0.1;
        public static double PLANELAUNCH = 0.9;
        //Servo
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
}

