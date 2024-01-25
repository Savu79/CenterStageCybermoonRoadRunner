package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MEEP2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-38.175,9.125,Math.toRadians(90)))
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

//        TrajectorySequence traj1L=
//                .build();
//        TrajectorySequence traj2L= drive.trajectorySequenceBuilder(new Pose2d(-38.175,9.125,Math.toRadians(90)))
//                .strafeRight(18.175)
//                .lineToLinearHeading(new Pose2d(-40.25, 55, Math.toRadians(-90)))
//                .addDisplacementMarker(() -> {
//                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
//                })
//                .addDisplacementMarker(() -> {
//                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
//                })
//                .addDisplacementMarker(() -> {
//                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
//                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
//                })
//                .build();

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}