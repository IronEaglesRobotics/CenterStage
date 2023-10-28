package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.robot = new Robot(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        // Set the drive input
        this.robot.getDrive().setInput(x, y, z);

        // Pickup
//        this.robot.getClaw().close();
//        this.robot.getClaw().open();
//        this.robot.getClaw().up();
//        this.robot.getClaw().down();

        // Gantry
//        this.robot.getGantry().up();
//        this.robot.getGantry().down();
//        this.robot.getGantry().armIn();
//        this.robot.getGantry().armOut();
//        this.robot.getGantry().intake();
//        this.robot.getGantry().deposit();
    }
}
