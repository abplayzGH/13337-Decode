package com.robozark.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double goalX = -30.0;    // TUNE
        double goalY = 30.0;     // TUNE
        double goalHeading = Math.toRadians(135);

        double parkX = 37.0;     // TUNE
        double parkY = -33.0;

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(60, -12, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(36, 30, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(55)
                .lineToY(30)

                .strafeTo(new Vector2d(11, 25))

                .strafeTo(new Vector2d(11, 56))

                .strafeTo(new Vector2d(11, 25))

                .strafeTo(new Vector2d(-11, 25))

                .strafeTo(new Vector2d(-11, 56))


//                .splineToSplineHeading(new Pose2d(goalX, goalY, goalHeading), Math.toRadians(90))
//
//                .splineToSplineHeading(new Pose2d(12, 20, Math.toRadians(90)), Math.toRadians(90))
//                .lineToY(55)
//                .lineToY(30)
//
//                .splineToSplineHeading(new Pose2d(goalX, goalY, goalHeading), Math.toRadians(90))
//
//                .splineToSplineHeading(new Pose2d(-16, 30, Math.toRadians(90)), Math.toRadians(90))
//                .lineToY(55)
//
//                .splineToSplineHeading(new Pose2d(goalX, goalY, goalHeading), Math.toRadians(90))
//                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(90))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}