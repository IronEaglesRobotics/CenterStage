package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-51.5, 33.6,Math.toRadians(180)))
//                                .lineToConstantHeading(new Vector2d(-45,59.5))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Pose2d(-48,59.5).vec(),Math.toRadians(0))
                                .lineToConstantHeading(new Pose2d(33,59.5).vec())
                                .splineToConstantHeading(new Pose2d(52.5, 36).vec(),Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity BlueFrontStage = new DefaultBotBuilder(meepMeep)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180),15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                                //Score then pick up 1 white
                                .lineToLinearHeading(new Pose2d(-39.7, 33.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-51.5, 33.6, Math.toRadians(180)).plus(new Pose2d(-5.4,-1.5))).waitSeconds(.01)

                                //Spline to board
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Pose2d(-33, 59.5, Math.toRadians(180)).vec(),Math.toRadians(0))
                                .lineToConstantHeading(new Pose2d(33, 59.5, Math.toRadians(180)).vec())
                                .splineToConstantHeading(new Pose2d(52, 34, Math.toRadians(8)).vec(),Math.toRadians(0))

                                //Go back to white stack
                                .splineToConstantHeading(new Pose2d(33, 59.5, Math.toRadians(180)).plus(new Pose2d(0,-2)).vec(), Math.toRadians(180))
                                .lineToConstantHeading(new Pose2d(-45, 59.5, Math.toRadians(180)).plus(new Pose2d(0,-2)).vec())
                                .splineToConstantHeading(new Pose2d(-51.5, 33.6, Math.toRadians(180)).plus(new Pose2d(-3.9, -3.7)).vec(), Math.toRadians(180))

                                //Go back to board
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Pose2d(-33, 59.5, Math.toRadians(180)).vec(),Math.toRadians(0))
                                .lineToConstantHeading(new Pose2d(33, 59.5, Math.toRadians(180)).vec())
                                .splineToConstantHeading(new Pose2d(52, 34, Math.toRadians(8)).vec(),Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity BlueFrontStage3 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180),15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                                //Score then pick up 1 white
                                .lineToLinearHeading(new Pose2d(-46.7, 39.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-46.7,50.5,Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-51.5, 33.6, Math.toRadians(180)).plus(new Pose2d(-5.4,-1.5))).waitSeconds(.01)

                                //Spline to board
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Pose2d(-33, 59.5, Math.toRadians(180)).vec(),Math.toRadians(0))
                                .lineToConstantHeading(new Pose2d(33, 59.5, Math.toRadians(180)).vec())
                                .splineToConstantHeading(new Pose2d(53.6, 32, Math.toRadians(8)).vec(),Math.toRadians(0))

                                //Park
                                .lineToLinearHeading(new Pose2d(45,58,Math.toRadians(-180)))
                                .build()
                );

        RoadRunnerBotEntity BlueFrontStage1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60,60,Math.toRadians(180),Math.toRadians(180),15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                                //Score then pick up 1 white
                                .lineToLinearHeading(new Pose2d(-36, 45.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-36,33.5,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-32,33.5,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-51.5, 33.6, Math.toRadians(180)).plus(new Pose2d(-5.4,-1.5))).waitSeconds(.01)

                                //Spline to board
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Pose2d(-33, 59.5, Math.toRadians(180)).vec(),Math.toRadians(0))
                                .lineToConstantHeading(new Pose2d(33, 59.5, Math.toRadians(180)).vec())
                                .splineToConstantHeading(new Pose2d(53.6, 32, Math.toRadians(8)).vec(),Math.toRadians(0))

                                //Park
                                .lineToLinearHeading(new Pose2d(45,58,Math.toRadians(-180)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(myBot)
                //.addEntity(BlueFrontStage)
                .addEntity(BlueFrontStage3)
                //.addEntity(BlueFrontStage1)
                .start();
    }
}