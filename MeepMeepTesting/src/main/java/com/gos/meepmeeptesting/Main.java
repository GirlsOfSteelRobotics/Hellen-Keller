package com.gos.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, -61, Math.toRadians(-90)))
                // START COPY AND PASTE
                .setReversed(true)
                .splineTo(new Vector2d(-35.5,-9), Math.toRadians(90))
                .strafeTo(new Vector2d(-45,-9))
                .strafeToConstantHeading(new Vector2d(-45,-55))
                .strafeTo(new Vector2d(-60,-60))
                .strafeTo(new Vector2d(-45, - 38))
                .strafeTo(new Vector2d(-45, -9))
                .strafeTo(new Vector2d(-56,-9))
                .strafeTo(new Vector2d(-56,-60))
                .strafeTo(new Vector2d(-56,-40))
                .strafeTo(new Vector2d(-56,-8))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(-53,-8))
                .strafeTo(new Vector2d(-53,-29))
                .strafeTo(new Vector2d(-63,-29))
                .strafeTo(new Vector2d(-63,-60))
                // END COPY AND PASTE





//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}