package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // M-Series Mac Graphics Acceleration
        System.setProperty("sun.java2d.metal", "true");
        System.setProperty("sun.java2d.opengl", "false");

        /* Put MeepMeep Code Below

        Remember -
        Default Bot Constraints:

        Max Velocity: 30in/s
        Max Acceleration: 30in/s/s
        Max Angular Velocity: 60deg/s
        Max Angular Acceleration: 60deg/s/s
        Track Width: 15in
        Bot Width: 18in
        Bot Height: 18in
        Start Position: (x: 0in, y: 0in, heading: 0rad)
        Color Scheme: Inherited from MeepMeep.colorManager unless overridden
        Drive Train Type: Mecanum
         */
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}