package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(10,34,Math.toRadians(360)))
                        .strafeRight(25)
                        .lineTo(new Vector2d(25,9))
                        .lineToLinearHeading(new Pose2d(47,38, Math.toRadians(180)))
                        .strafeLeft(28)
                        .lineTo(new Vector2d(61, 10))
                        .waitSeconds(1)
                        .addDisplacementMarker(()->{//functioneaza dupa 0.5 secunde
                            //pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                            //robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                        })
                        .addDisplacementMarker(()->{//functioneaza duoa strafe(10)
                            //robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            //pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);//(new Pose2d(-43.75, 53.875, Math.toRadians()));
                        })
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}