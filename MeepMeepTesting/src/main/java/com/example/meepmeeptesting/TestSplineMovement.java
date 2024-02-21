package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestSplineMovement {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
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
                        .addEntity(myBot)
                        .start();




    }
}