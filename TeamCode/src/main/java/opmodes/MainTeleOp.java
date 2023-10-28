package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        this.robot = new Robot(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        this.robot.getDrive().setInput(x, y, z);
    }
}
