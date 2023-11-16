package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "Drivebase Only", group = "OpModes")
public class Drivebase extends OpMode {
    //turbo mode
    public static double normal = 1;
    public static double turbo = 0.5;
    //create robot instance
    private Robot robot;
    //create controller 1 (driver)
    private Controller controller1;
    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        controller1 = new Controller(gamepad1);
        telemetry.addLine("Started");
        //update to make sure we receive last line of code
        telemetry.update();
    }

    @Override
    public void loop() {
        //create and set X, Y, and Z for drive inputs
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double z = -gamepad1.right_stick_x;
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));

        //turbo activation
        if (controller1.getA().isJustPressed()){
            x *= turbo;
            y *= turbo;
            z *= turbo;
        } else if (controller1.getA().isJustReleased()){
            x *= normal;
            y *= normal;
            z *= normal;
        }
    }
}
