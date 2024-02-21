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

        AutoRedFar1PL,
        AutoRedFar1PC ,
        AutoRedFar1PR,
        AutoRed2PL,
        AutoRed1PRSPlINE,
        AutoRedNear2PR,
        TEST,

    }
    static cas currentState=cas.AutoRedNear2PR;//TODO shimba asta in functie de ce caz vrei
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
            case AutoBlue2PL:
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


            case AutoRedFar1PL:
                RoadRunnerBotEntity myBotRedFar1PL = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                        .lineTo(new Vector2d(-35, -34))
                                        .turn(Math.toRadians(90))
                                        .back(5)
                                        .waitSeconds(1)
                                        .forward(5)
                                        .strafeRight(25)
                                        .lineTo(new Vector2d(25, -9))
                                        .lineToLinearHeading(new Pose2d(47, -28, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .strafeRight(18)
                                        .lineTo(new Vector2d(59, -10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBotRedFar1PL)
                        .start();
                break;

            case AutoRedFar1PC:
                RoadRunnerBotEntity myBotRedFar1PC = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                        .lineTo(new Vector2d(-35, -38))
                                        .waitSeconds(1)
                                        .strafeLeft(20)
                                        .lineTo(new Vector2d(-52, -9))
                                        .lineToLinearHeading(new Pose2d(25, -9, Math.toRadians(180)))
                                        .lineTo(new Vector2d(47, -34))
                                        .waitSeconds(1)
                                        .strafeRight(24)
                                        .lineTo(new Vector2d(59, -10))
                                        .waitSeconds(1)
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBotRedFar1PC)
                        .start();
                break;

            case AutoRed1PRSPlINE: // case pentru teste
                RoadRunnerBotEntity AutoRed1PRSPlINE = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(51, -37, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .forward(3)
                                        .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(330))
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(AutoRed1PRSPlINE)
                        .start();
                break;

            case AutoRedNear2PR:
                RoadRunnerBotEntity AutoRedNear2PR = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(270)))
                                .lineTo(new Vector2d(-40, 37))
                                .addDisplacementMarker(() ->{
                                    //robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                                })
                                .turn(Math.toRadians(90))
                                .addDisplacementMarker(() ->{
                                    //robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                                    //robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                                })
                                .strafeRight(25)
                                .lineTo(new Vector2d(25, 9))
                                .addDisplacementMarker(() ->{
                                    //robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                                    //pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                                })
                                .lineToLinearHeading(new Pose2d(43, 38, Math.toRadians(180)))
                                .addDisplacementMarker(() ->{
                                    //robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                                    //pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                                })
                                .waitSeconds(1)
                                .forward(4)
                                .strafeLeft(25)
                                .lineTo(new Vector2d(52, 13))
                                .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(AutoRedNear2PR)
                        .start();
                break;
        }


    }
}

/*case AutoRedNear2PR:
                RoadRunnerBotEntity AutoRedNear2PR = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(90)))
                                        .addDisplacementMarker(() -> {
//                                            robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
//                                            pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                                        })
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(10, -32))
                                        .turn(Math.toRadians(-90))
                                        .back(3)
                                        .addDisplacementMarker(() -> {
//                                            robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
//                                            pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                                        })
                                        .waitSeconds(1)
                                        .forward(3)
                                        .addDisplacementMarker(() -> {
//                                            robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                                        })
                                        .strafeLeft(23)
                                        .addDisplacementMarker(() -> {
//                                            pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                                        })
                                        .lineToLinearHeading(new Pose2d(-70, -9, Math.toRadians(180)))
                                        .build());
                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(AutoRedNear2PR)
                        .start();
                break;
 */

/*
L
drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(90)))
        .lineToLinearHeading(new Pose2d(13, -34, Math.toRadians(180)))
        .lineToLinearHeading(new Pose2d(51, -24, Math.toRadians(180)))
        .waitSeconds(1)
        .forward(3)
        .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(330))
        .build());

        C
        drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(90)))
                                        .forward(25)
                                        .lineToLinearHeading(new Pose2d(51, -32, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .forward(3)
                                        .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(330))
                                        .build());

                                        R
 */