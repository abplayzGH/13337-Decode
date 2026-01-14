package com.robozark.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
        final Vector2d SPIKE_3 = new Vector2d(-12, 25);
        final Vector2d SPIKE_2 = new Vector2d(12, 25);
        final Vector2d SPIKE_1 = new Vector2d(36, 25);
        final Vector2d SPIKE_3_FINAL = new Vector2d(-12, 50);
        final Vector2d SPIKE_2_FINAL = new Vector2d(12, 50);
        final Vector2d SPIKE_1_FINAL = new Vector2d(36, 50);
//        final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));
        final Pose2d START_POSE = new Pose2d(-49, 49, Math.toRadians(305));

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(START_POSE)

                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90))

                .strafeToLinearHeading(SPIKE_3, Math.toRadians(90))

                .strafeTo(SPIKE_3_FINAL)

                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90))

                .strafeToLinearHeading(SPIKE_2, Math.toRadians(90))

                .strafeTo(SPIKE_2_FINAL)

                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90))

                .strafeToLinearHeading(SPIKE_1, Math.toRadians(90))

                .strafeTo(SPIKE_1_FINAL)

                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}