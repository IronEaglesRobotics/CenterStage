package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous (name= "varun_is_Dumb")
public class vannobAuto_RightRed extends LinearOpMode {
    public Robot robot;
    int teamElementLocation = 2;
    int macrostate = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        // make trajectories

        while (!(isStarted() || isStopRequested())) {
            double currentRuntime = getRuntime();
            robot.update(currentRuntime);

//            int newLocation = robot.camera.getMarkerId();
//            if (newLocation != -1) {
//                teamElementLocation = newLocation;
//            }

            telemetry.addLine("Initialized");

            telemetry.addLine("Randomization of T.O.M: "+teamElementLocation);
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }

        while(macrostate < 6 || !isStopRequested()){
            //update everything
            robot.update(getRuntime());

            switch (macrostate){
                case (0):
                    //follow trajectory
                    macrostate++;
                    break;
                case (1):
                    // run intake
                    macrostate++;
                    break;
                case (2):
                    macrostate++;
                    break;
                case (3):
                    macrostate++;
                    break;
                case (4):
                    macrostate++;
                    break;
                case (5):
                    macrostate++;
                    break;
            }
        }
    }
}
