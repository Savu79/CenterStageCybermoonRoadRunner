package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    enum cas
    {
        AutoBlue1PL,
        AutoBlue1PR,
        AutoBlue1PC,
        AutoBlue2PL,
        AutoRed1PL,
        AutoRed2PL,
    }
    static cas currentState=cas.AutoBlue1PC;//TODO shimba asta in functie de ce caz vrei
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        switch (currentState) {
            case AutoBlue1PL:
                RoadRunnerBotEntity myBot1L = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(10, 34, Math.toRadians(360)))
                                        .strafeRight(25)
                                        .lineTo(new Vector2d(25, 9))
                                        .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)))
                                        .strafeLeft(28)
                                        .lineTo(new Vector2d(61, 10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot1L)
                        .start();
                break;
            case AutoBlue1PR:
                RoadRunnerBotEntity myBot1R = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(10, 34, Math.toRadians(180)))
                                        .strafeLeft(25)
                                        .lineTo(new Vector2d(25, 9))
                                        .lineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)))
                                        .strafeLeft(18)
                                        .lineTo(new Vector2d(61, 10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot1R)
                        .start();
                break;
            case AutoBlue1PC:
                RoadRunnerBotEntity myBot1C = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                        .lineTo(new Vector2d(10, 34))
                                        .lineToLinearHeading(new Pose2d(47, 34, Math.toRadians(180)))
                                        .strafeLeft(24)
                                        .lineTo(new Vector2d(61, 10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot1C)
                        .start();
                break;
            case  AutoBlue2PL:
                RoadRunnerBotEntity myBot2L = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(10, 34, Math.toRadians(360)))
                                        //TODO punem bratul jos, dechidem servo 1 si servo control la min
                                        .waitSeconds(2)
                                        //TODO punem bratul la mid, servo la max
                                        .strafeRight(25)
                                        .lineToLinearHeading(new Pose2d(-54, 9, Math.toRadians(180)))
                                        //TODO bratul jos, inchidem intake 1 control servo la min
                                        .waitSeconds(2)
                                        //TODO brat la mid control servo la max
                                        .lineTo(new Vector2d(25, 9))
                                        //TODO brat la max
                                        .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)))
                                        //TODO deschidem intake 1 si 2
                                        .waitSeconds(1)
                                        //TODO bratul il punem la mid
                                        .strafeLeft(28)
                                        .lineTo(new Vector2d(61, 10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot2L)
                        .start();
                break;

        }
    }
}