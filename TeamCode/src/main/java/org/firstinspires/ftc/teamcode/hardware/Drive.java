//package org.firstinspires.ftc.teamcode.hardware;
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
//
//import java.util.Arrays;
//
//public class Drive {
//
//    DcMotor frontLeft;
//    DcMotor frontRight;
//    DcMotor backLeft;
//    DcMotor backRight;
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
//
//    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
//    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);
//    private TrajectorySequenceRunner trajectorySequenceRunner;
//
//    public Drive(HardwareMap hardwareMap) {
//        this.init(hardwareMap);
//    }
//
//    private void init(HardwareMap hardwareMap) {
//        this.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.backRight = hardwareMap.get(DcMotor.class, "backRight");
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//    }
//
//    public void setInput(Gamepad gamepad1,Gamepad gamepad2) {
//        float x = -gamepad1.left_stick_x;
//        float y = gamepad1.left_stick_y;
//        float z = -gamepad1.right_stick_x;
//        float flPower = x + y + z;
//        float frPower = -x + y - z;
//        float blPower = -x + y + z;
//        float brPower = x + y - z;
//
//        float max = Math.max(1, Math.max(Math.max(Math.abs(blPower), Math.abs(brPower)), Math.max(Math.abs(flPower),Math.abs(frPower))));
//        float scale = 1-((max-1) / max);
//        flPower *= scale;
//        frPower *= scale;
//        blPower *= scale;
//        brPower *= scale;
//
//        frontLeft.setPower(flPower);
//        backLeft.setPower(blPower);
//        frontRight.setPower(frPower);
//        backRight.setPower(brPower);
//    }
//
//    public void update() {
//        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//        if (signal != null) setDriveSignal(signal);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
//    }
//
//    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
//        return new MinVelocityConstraint(Arrays.asList(
//                new AngularVelocityConstraint(maxAngularVel),
//                new MecanumVelocityConstraint(maxVel, trackWidth)
//        ));
//    }
//
//    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
//        return new ProfileAccelerationConstraint(maxAccel);
//    }
//}
