package com.robozark.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        double GOAL_HEADING = Math.toRadians(135);
        final Vector2d GOAL = new Vector2d(-36, 30);
        final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
        final Vector2d PARK = new Vector2d(37, -33);
        final Vector2d SPIKE_3 = new Vector2d(-12, 20);
        final Vector2d SPIKE_2 = new Vector2d(12, 25);
        final Vector2d SPIKE_1 = new Vector2d(36, 25);
        final Vector2d SPIKE_3_FINAL = new Vector2d(-12, 50);
        final Vector2d SPIKE_2_FINAL = new Vector2d(12, 50);
        final Vector2d SPIKE_1_FINAL = new Vector2d(36, 50);
//        final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));
        final Pose2d START_POSE = new Pose2d(-49, 49, Math.toRadians(125));
        final Pose2d START_POSE2 = new Pose2d(55, 10, Math.toRadians(180));

//        final Pose2d Gate = new Pose2d(-2, 45, Math.toRadians(90));
        final Vector2d Gate = new Vector2d(10, 55);
        double GOAL_BACk_HEADING = Math.toRadians(135);
        final Vector2d GOAL_BACK = new Vector2d(-36, 30);
        final Pose2d GOAL_BACK_POSE = new Pose2d(GOAL_BACK, GOAL_BACk_HEADING);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18) // width, height in inches
                .build();

        // GOAL
//        myFirstBot.runAction(
//                myFirstBot.getDrive().actionBuilder(START_POSE)
//
//                        // Preload score
//                        .setTangent(GOAL_HEADING)
//                        .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
//
//                        //Go to Middle Balls
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0))
//
//                        //Intake Middle Balls
//                        .setTangent(Math.toRadians(90))
//                        .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90))
//
////                        // Return to goal smoothly
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
////
////                        // Open gate
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
//
//                        //Return to goal
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
//
//                        // Open gate
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
//
//                        //Return to goal
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
//
//                        // Open go to spike 3
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(SPIKE_3, Math.toRadians(90)), Math.toRadians(90))
//
//                        // Intake spike 3
//                        .setTangent(Math.toRadians(90))
//                        .splineToConstantHeading(SPIKE_3_FINAL, Math.toRadians(90))
//
//
//                        //Return to goal
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
//
//                        .build()
//        );

        myFirstBot.runAction(
                myFirstBot.getDrive().actionBuilder(new Pose2d(60, 10, Math.toRadians(180)))


                        .splineToLinearHeading(new Pose2d(SPIKE_1_FINAL, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(START_POSE2, Math.toRadians(270))

                        .splineToLinearHeading(new Pose2d(new Vector2d(55, 55), Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(START_POSE2, Math.toRadians(270))

                        .splineToLinearHeading(new Pose2d(new Vector2d(50, 55), Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(START_POSE2, Math.toRadians(270))

                        .splineToLinearHeading(new Pose2d(new Vector2d(45, 55), Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(START_POSE2, Math.toRadians(270))




                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}