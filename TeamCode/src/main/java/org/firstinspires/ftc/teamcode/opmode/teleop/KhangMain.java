package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.ArmPos;
import org.firstinspires.ftc.teamcode.hardware.DoorPos;
import org.firstinspires.ftc.teamcode.hardware.Down;
import org.firstinspires.ftc.teamcode.hardware.Hieght;
import org.firstinspires.ftc.teamcode.hardware.HopperPos;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.SlidePos;

@Config
@TeleOp(name = "Meet 1 TeleOp", group = "OpModes")
public class KhangMain extends OpMode {

    //create and set start position of intake
    private Intake.Position Currentpos = Intake.Position.UP;
    //inform if slides are in lowest position or not
    private Down.DownArm DownQuestion = Down.DownArm.YES;
    //create and set start position of slides
    private SlidePos.SPosition CurrentSpos = SlidePos.SPosition.DOWN;
    //create and set start position of arm
    private ArmPos.APosition CurrentApos = ArmPos.APosition.SDOWN;
    //create and set start position of door, default close
    private DoorPos.DoorPosition CurrentDpos = DoorPos.DoorPosition.CLOSE;
    //create and set start position of hopper
    private HopperPos.hopperPos hopperpos = HopperPos.hopperPos.DOWN;
    //create manual slide height varable and set it to hold position as start
    private Hieght.height CurrentHeight = Hieght.height.HOLD;
    //create robot instance
    private Robot robot;
    //create servo for plane
    private Servo planeServo;
    //create controller 1 (driver)
    private Controller controller1;
    //create controller 2 (macro user and one with hard life)
    private Controller controller2;
    //set starting slide height to 0
    private double sHeight = 0;

    @Override
    public void init() {

        //make each servo, motor, and controller and configure them
        planeServo = hardwareMap.get(Servo.class, "Plane Servo");
        this.robot = new Robot(hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        //feedback to driver hub to know if init was successful
        telemetry.addLine("Started");
        //update to amke sure we receive last lien of code
        telemetry.update();
    }

    @Override
    public void loop() {

        //create and set X, Y, and Z for drive inputs
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double z = -gamepad1.right_stick_x;
            robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));

        //set intake to be pressure reactant to right trigger
        robot.intake.setDcMotor(gamepad2.right_trigger);
        //create variable for door open or close to open and close in sync with intake
        double door = gamepad2.right_trigger;

        //manual slide height control
        double sHeight = gamepad2.right_stick_y;
        //set slides all the way down aka reset button
        double intakeON = gamepad2.right_trigger;
        //graph input of Y joystick to make macros for slides
        double sY = -gamepad2.left_stick_y;
        //graph of X
        double sX = gamepad2.left_stick_x;

        //variable to determine shoot drone or not
        double shoot = gamepad1.right_trigger;

        //update variables to constantly feed position each component should be in
            //intake
        robot.intake.setpos(Currentpos);
            //slide macro
        robot.slides.setSPos(CurrentSpos);
            //slide manual
//        robot.slides.setHeight(CurrentHeight);
            //arm position
        robot.arm.setPos(CurrentApos);
            //door open or not
        robot.arm.openDoor(CurrentDpos);
            //hopper position
        robot.arm.setHopper(hopperpos);
            //update
        controller2.update();

        //manual slide input reader
//        if (sHeight >= 0.2) {
//            CurrentHeight = Hieght.height.ASCEND;
//        } else if (sHeight <= -0.2) {
//            CurrentHeight = Hieght.height.DESCEND;
//        } else {
//            CurrentHeight = Hieght.height.HOLD;
//        }

        //read values to determine if plane should shoot or not
        if (shoot >= 0.9) {
            planeServo.setPosition(0.2);
        } else {
            planeServo.setPosition(0);
        }

        //make door rise as intake goes on
        if (intakeON >= 0.01) {
            CurrentDpos = DoorPos.DoorPosition.OPEN;
        } else {
            CurrentDpos = DoorPos.DoorPosition.CLOSE;
        }

        //control position of hopper
        if (controller2.getLeftBumper().isJustPressed()) {
            hopperpos = HopperPos.hopperPos.UP;
        } else if (controller2.getLeftBumper().isJustReleased()) {
            hopperpos = HopperPos.hopperPos.DOWN;
        }

        //shift intake up one pixel
        if (controller2.getDLeft().isJustPressed()) {
            //prevent from going higher than highest legal value
            if (Currentpos != Intake.Position.UP) {
                Currentpos = Currentpos.nextPosition();
            }
        }

        //shift intake down one pixel
        if (controller2.getDRight().isJustPressed()) {
            //prevent from going lower than lowest value
            if (Currentpos != Intake.Position.STACK1) {
                Currentpos = Currentpos.previousPosition();
            }
        }

        //set intake to max position
        if (controller2.getDUp().isJustPressed()) {
            Currentpos = Currentpos.UP;
        }

        //set intake to lowest position
        if (controller2.getDDown().isJustPressed()) {
            Currentpos = Currentpos.STACK1;
        }

        //arm safety pause going up
        if (controller2.getA().isJustPressed()){
            CurrentApos = CurrentApos.SAFTEYUP;
            DownQuestion = DownQuestion.NO;
        }

        //arm safety pause going down
        if (controller2.getX().isJustPressed()) {
            CurrentApos = CurrentApos.SAFTEYDOWN;
            DownQuestion = DownQuestion.NO;
        }

        //arm all the way up
        if (controller2.getB().isJustPressed()) {
            CurrentApos = CurrentApos.SCORE;
            DownQuestion = DownQuestion.NO;
        }

        //arm all the way down
        if (controller2.getY().isJustPressed()) {
            CurrentApos = CurrentApos.SDOWN;
            DownQuestion = DownQuestion.YES;
        }

        if (sY >= 0.2) {
            CurrentSpos = CurrentSpos.MAX;
        }

        if (sY <= -0.2) {
            CurrentSpos = CurrentSpos.TAPE1;
        }

        if (sX >= 0.2) {
            CurrentSpos = CurrentSpos.TAPE3;
        }

        if (sX <= -0.2) {
            CurrentSpos = CurrentSpos.TAPE2;
        }

        if (gamepad2.left_trigger >= 0.35) {
            CurrentSpos = CurrentSpos.DOWN;
        }

//        if (gamepad2.left_stick_button = true) {
//            CurrentSpos = CurrentSpos.DOWN;
//        }
    }
}
