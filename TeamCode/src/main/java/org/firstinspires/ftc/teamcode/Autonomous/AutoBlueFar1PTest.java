package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.OpenCVCode.CenterStagePipelineBlue;
import org.firstinspires.ftc.teamcode.OpenCVCode.CenterStagePipelineRed;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Teste.TestSleeveDetectionBlue;

@Autonomous
public class AutoBlueFar1PTest extends LinearOpMode {
    //*1P de la un pixel
    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    private CenterStagePipelineBlue.CenterStagePosition pi;
    int extTarget=0;
    Pose2d myPose;
    int pivTarget=RobotHardware.PivotINIT;
    boolean isClosed=false;
    boolean afost=false;
    OpenCvCamera backCamera;
    CenterStagePipelineBlue pipeline;
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void runOpMode(){
        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        pivMotor = new PivotingMotorSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new CenterStagePipelineBlue();
        backCamera.setPipeline(pipeline);

        backCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(backCamera, 10);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //*Dupa declaratii ne asiguram ca robotul se afla in dimensiuni:

        pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);

        robot.MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
        robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
        robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);

        drive.setPoseEstimate(new Pose2d(-36, 61, Math.toRadians(270)));

        //* While pt ca pivMotor sa ajunga la pozitie + detectie
        while(opModeInInit()){
            pivMotor.update();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            pi=pipeline.getAnalysis();
        }
        RobotHardware.State currentState;
        currentState= RobotHardware.State.TRAJECTORY_1;


        //* Traiectorii RoadRunner

        //! pt fiecare caz, deplasarea pana la depunerea primului pixel
        //? LEFT
        TrajectorySequence traj1L= drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(270)))
                .lineTo(new Vector2d(-43, 34))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .turn(Math.toRadians(90))
                .forward(7)
                .waitSeconds(0.5)
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .strafeRight(25)
                .lineTo(new Vector2d(25, 12))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                })
                .lineToLinearHeading(new Pose2d(47, 44, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                })
                .waitSeconds(0.5)
                .back(0.5)
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .waitSeconds(1)
                .strafeLeft(31)
                .lineTo(new Vector2d(56, 13))
                .build();

        TrajectorySequence traj1C= drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(270)))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .strafeRight(3)
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(360)))
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .lineTo(new Vector2d(25, 12))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                })
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                })
                .waitSeconds(0.5)
                .back(0.5)
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .waitSeconds(1)
                .strafeLeft(23)
                .lineTo(new Vector2d(56, 13))
                .build();

        TrajectorySequence traj1R= drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(270)))
                .lineTo(new Vector2d(-34, 34))
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .waitSeconds(1)
                .strafeLeft(22)
                .lineTo(new Vector2d(25, 12))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                })
                .lineToLinearHeading(new Pose2d(47, 32.5, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                })
                .waitSeconds(0.5)
                .back(0.5)
                .addDisplacementMarker(() ->{
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);
                })
                .waitSeconds(1)
                .strafeLeft(19)
                .lineTo(new Vector2d(56, 13))
                .build();


        while(opModeIsActive()){
            drive.update();
            switch (pi) {
                case LEFT:
                    if(!afost) {
                        drive.followTrajectorySequenceAsync(traj1L);
                        afost=true;
                    }
                    break;
                case CENTER:
                    if(!afost) {
                        drive.followTrajectorySequenceAsync(traj1C);
                        afost=true;
                    }
                    break;

                case RIGHT:
                    if(!afost) {
                        drive.followTrajectorySequenceAsync(traj1R);
                        afost=true;
                    }
                    break;
            }
            pivMotor.updateAuto();
            myPose=drive.getPoseEstimate();
            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.update();
        }
    }
}

