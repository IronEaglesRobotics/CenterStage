//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import static org.firstinspires.ftc.teamcode.opmode.teleop.Team.BLUE;
//import static org.firstinspires.ftc.teamcode.opmode.teleop.Team.RED;
//import static java.lang.Math.PI;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.controller.Controller;
//import org.firstinspires.ftc.teamcode.hardware.Arm;
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.hardware.robby.Slides;
//
//@Config
//public abstract class AbstractTeleOp extends OpMode {
//    private Robot robot;
//    public Team team;
//    Controller driver1;
//    Controller driver2;
//
//    public static double normal = 0.5;
//    public static double vazrooooming = 1.0;
//    public static int heightIncrement = 20;
//
//    Pose2d robot_pos;
//    double robot_x, robot_y, robot_heading;
//
//    // auto align variables
//    double headingPID;
//    public static double headingP = 0.01;
//    public static double headingI = 0.03;
//    public static double headingD = 0.0005;
//    public static PIDController headingController = new PIDController(headingP, headingI, headingD);
//
//    double strafePID;
//    public static double strafeP = 0.05;
//    public static double strafeI = 0;
//    public static double strafeD = 0.01;
//    public static PIDController strafeController = new PIDController(strafeP, strafeI, strafeD);
//
//    double robot_y_pos;
//    boolean fixed90Toggle = false;
//    boolean fixed0Toggle = false;
//
////    public static double robot_width = 12;
////    public static double robot_length = 12;
////    public static double robot_radius = 6;
////
////    public static double groundJuncRadius = 6;
////    public static double coneRadius = 4;
////
////    public static double armWait = 0.2;
//
////    private double timeSinceOpened = 0; // for claw
////    private double timeSinceClosed = 0;
//
//    //    private int delayState = 0; // for arm
////    private double delayStart = 0;
////    private boolean doArmDelay = false; // for arm
//    private boolean isAutoClose = true;
//
//    @Override
//    public void init() {
//        robot =  new Robot(hardwareMap);
//        driver1 = new Controller(gamepad1);
//        driver2 = new Controller(gamepad2);
//        if (team == RED) {
//            driver1.setColor(255, 0, 0);
//            driver2.setColor(255, 0, 0);
//        } else if (team == BLUE) {
//            driver1.setColor(0, 0, 255);
//            driver2.setColor(0, 0, 255);
//        }
//
//        PoseStorage.AutoJustEnded = false;
//    }
//
//    @Override
//    public void init_loop() {
//        ////init when camera is on and responding
////        if (robot.camera.getFrameCount() < 1) {
////            telemetry.addLine("Initializing Robot...");
////        } else {
////            telemetry.addLine("Initialized");
////        }
//        telemetry.addLine("Initialized");
//        telemetry.addLine(robot.getTelemetry());
//        telemetry.update();
//    }
//
//    //find out what quarter you are in
//    private int getQuadrant(double angle) {
//        if (0 < angle && angle < PI/2.0) {
//            return 1;
//        } else if (PI/2.0 < angle && angle < PI) {
//            return 2;
//        } else if (PI < angle && angle < 3*PI/2.0) {
//            return 3;
//        } else {
//            return 4;
//        }
//    }
//
//    @Override
//    public void loop() {
//        // robot position update
//        robot_pos = robot.drive.getPoseEstimate();
//        robot_x = robot_pos.getX();
//        robot_y = robot_pos.getY();
//        robot_heading = robot_pos.getHeading(); // in radians
//
//        // driver 1 controls
//        driver1.update();
//        driver2.update();
//
//        //create and set X, Y, and Z for drive inputs
//        double x = -gamepad1.left_stick_y;
//        double y = -gamepad1.left_stick_x;
//        double z = -gamepad1.right_stick_x;
//
////         figure out if a toggle is present
//        if (driver1.getRightBumper().isPressed()) {
//            if (!fixed90Toggle) {
//                robot_y_pos = robot.drive.getPoseEstimate().getX();
//            }
//            fixed90Toggle = true;
//        } else {
//            fixed90Toggle = false;
//        }
//
//        if (!fixed90Toggle && driver1.getLeftBumper().isPressed()) {
//            if (!fixed0Toggle) {
//                robot_y_pos = robot.drive.getPoseEstimate().getY();
//            }
//            fixed0Toggle = true;
//        } else {
//            fixed0Toggle = false;
//        }
//
//        // heading pid
//        headingController.setPID(headingP, headingI, headingD);
//        int quadrant = getQuadrant(robot_heading);
//        if (fixed0Toggle) {
//            switch (quadrant) {
//                case 1:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 0);//- right
//                    break;
//                case 2:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 180);//+ left
//                    break;
//                case 3:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 180);//- right
//                    break;
//                case 4:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 360);//+ left
//                    break;
//            }
//        } else if (fixed90Toggle) {
//            switch (quadrant) {
//                case 1:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 90);//- right
//                    break;
//                case 2:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 90);//+ left
//                    break;
//                case 3:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 270);//- right
//                    break;
//                case 4:
//                    headingPID = headingController.calculate(Math.toDegrees(robot_heading), 270);//+ left
//                    break;
//            }
//        } else {
//            headingPID = 0;
//        }
//
//        // strafe pid
//        strafeController.setPID(strafeP, strafeI, strafeD);
//        if (fixed0Toggle) {
//            strafePID = strafeController.calculate(robot_y, robot_y_pos);
//        } else if (fixed90Toggle) {
//            strafePID = strafeController.calculate(robot_x, robot_y_pos);
//        } else {
//            strafePID = 0;
//        }
//
//        telemetry.addLine("Wanted Position: "+robot_y_pos);
//        telemetry.addLine("Actual Position: "+robot_y);
//        telemetry.addLine("PID: "+strafePID);
//
//        // turbo
//        if (driver1.getLeftBumper().isPressed() || driver1.getRightBumper().isPressed()) {
//            x *= vazrooooming;
//            y *= vazrooooming;
//            z *= vazrooooming;
//        } else {
//            x *= normal;
//            y *= normal;
//            z *= normal;
//        }
//
//        // actually set power
//        if (fixed0Toggle || fixed90Toggle) {
//            robot.drive.setWeightedDrivePower(new Pose2d(x, 0, headingPID));
//        } else {
//            robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
//        }
//
//        // driver 2 controls
//        // increment heights
//        if (driver2.getDUp().isJustPressed()) {
//            Slides.heightOffset += heightIncrement;
//            robot.slides.setTarget(robot.slides.getTarget() + heightIncrement);
//        } else if (driver2.getDDown().isJustPressed()) {
//            Slides.heightOffset -= heightIncrement;
//            robot.slides.setTarget(robot.slides.getTarget() - heightIncrement);
//        }
//
//        switch (robot.runningMacro) {
//            case(0): // manual mode
//
//                //track pad is rad
//                // left right dpad is turn wrist
//                robot.slides.increaseTarget(driver2.getLeftStick().getY());
////                if (Math.abs(driver2.getRightStick().getY()) > 0.05) { // close claw if anything is moved
////                    robot.claw.close();
////                }
//
////                if (driver2.getRightBumper().isJustPressed()) {
////                    robot.arm.goToScore();
////                }
////                if (driver2.getLeftBumper().isJustPressed()) {
////                    robot.arm.goToIntake();
////                }
//
//                // retract all the time
//                if (driver2.getLeftStickButton().isJustPressed()) {
//                    robot.runningMacro = 5;
//                }
//
//                // to cancel macro and allow manual movement
////                if (driver2.getLeftStick().getY() > 0.05 || driver2.getRightStick().getY() > 0.05) {
////                    robot.runningMacro = 0;
////                    robot.lastMacro = 0;
////                }
//
//                if (driver2.getTouchpad().isJustPressed()) {
//                    robot.runningMacro = 4;
//                } else if (driver2.getX().isJustPressed()) { // high position [closed, bring up, bring out]
//                    robot.runningMacro = 3;
//                } else if (driver2.getY().isJustPressed()) { // middle position [middle goal level]
//                    robot.runningMacro = 2;
//                } else if (driver2.getB().isJustPressed() && !driver2.getStart().isJustPressed()) { // low position [low goal level]
//                    robot.runningMacro = 1;
//                } else if (driver2.getA().isJustPressed()) {
//                    if (robot.lastMacro == 0 || robot.lastMacro == 4) { // if not running any macros or picking up cones
//                    } else { // otherwise, I need to undo a macro
//                        robot.runningMacro = 5;
//                    }
//                } else {
//                    if (driver2.getRightBumper().isJustPressed()) {
//                        isAutoClose = false;
//                    }
//                    if (driver2.getLeftBumper().isJustPressed()) {
//                    }
//                }
//                break;
//            case(1):
////                robot.extendMacro(Slides.Position.LOW, Arm.APosition.SCORE, getRuntime());
//                break;
//            case(2):
////                robot.extendMacro(Slides.Position.MEDIUM, Arm.APosition.SCORE, getRuntime());
//                break;
//            case(3):
////                robot.extendMacro(Slides.Position.HIGH, Arm.APosition.SCORE, getRuntime());
//                break;
//            case(4):
////                robot.extendMacro(Slides.Position.PICKUP, Arm.APosition.SDOWN, getRuntime());
//                break;
//            case (5): // macro reset
//                robot.resetMacroNoDunk(0, getRuntime());
//                driver1.rumble(100);
//                break;
//        }
//
//        // cancel the macros
//        if (driver2.getRightStickButton().isJustPressed()) {
//            robot.runningMacro = 0;
//            robot.lastMacro = 0;
//            robot.macroState = 0;
//            robot.slides.cancel();
//        }
//
//        // update and telemetry
//        robot.update(getRuntime());
//
//        telemetry.addLine(robot.getTelemetry());
//        telemetry.addLine(String.format("Last Macro: %s\nRunning Macro: %s\nMacroState: %s", robot.lastMacro, robot.runningMacro, robot.macroState));
//        telemetry.update();
//    }
//}