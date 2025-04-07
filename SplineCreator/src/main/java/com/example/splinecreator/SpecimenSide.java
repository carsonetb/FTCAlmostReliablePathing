package com.example.splinecreator;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenSide {
    static Vector2d block1_pos = new Vector2d(-48.85, -37.46);
    static Vector2d block2_pos = new Vector2d(-57, -37.47);
    static Vector2d block3_pos = new Vector2d(-58.7, -36.3);
    static Vector2d basket_pos = new Vector2d(-59.23, -51.3);
    static double forward = 1.5708;
    static double left = 0;
    static double right = 3.1415;
    static double back = 3.1415 + 1.5708;
    static double basket_dir = 2.35619449019 + Math.PI / 2.0;
    static double block3_heading = 2.159;

    static double deg2rad(double deg) {
        return deg / (180/Math.PI);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(810);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 35, 20, 10, 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, forward))
                        .lineTo(new Vector2d(0, -33))
                        .waitSeconds(2)
                        .lineTo(new Vector2d(-block1_pos.getX(), block1_pos.getY()))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(0, -40, right))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-block2_pos.getX(), block2_pos.getY(), forward))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(0, -43, right))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(-block3_pos.getX(), block3_pos.getY(), -block3_heading + deg2rad(180)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(0, -46, right))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(50, -40, back))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(0, -49, right))
                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}