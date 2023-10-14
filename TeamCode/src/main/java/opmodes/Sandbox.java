package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/*
 * Demonstrates an empty iterative OpMode
 */
@TeleOp(name = "Sandbox", group = "Experimental")
public class Sandbox extends OpMode {

    private Camera camera;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        this.camera = new Camera(this.hardwareMap);
        this.camera.initTargetingCamera();
    }

    @Override
    public void loop() {

    }
}
