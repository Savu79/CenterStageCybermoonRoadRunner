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
public class AutoRedFarSimple extends LinearOpMode {
    //*1P de la un pixel
    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    private CenterStagePipelineRed.CenterStagePosition pi;
    int extTarget=0;
    Pose2d myPose;
    int pivTarget=RobotHardware.PivotINIT;
    boolean isClosed=false;
    boolean afost=false;
    OpenCvCamera backCamera;
    CenterStagePipelineRed pipeline;
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void runOpMode(){
        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        pivMotor = new PivotingMotorSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new CenterStagePipelineRed();
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

        pivMotor.setPivotingMotorTarget(RobotHardware.PivotINIT);

        robot.MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
        robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
        robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

        drive.setPoseEstimate(new Pose2d(-36, -61, Math.toRadians(90)));

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
        TrajectorySequence traj1L= drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
                .lineTo(new Vector2d(-36, -34))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                })
                .back(4)
                .waitSeconds(1)
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                })
                .forward(4)
                .strafeRight(25)
                .build();

        TrajectorySequence traj1C= drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                })
                .lineTo(new Vector2d(-36, -38))
                .waitSeconds(1)
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                })
                .back(4)
                .strafeLeft(20)
                .build();

        TrajectorySequence traj1R= drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
                .lineTo(new Vector2d(-36, -34))
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() ->{
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                })
                .back(4)
                .addDisplacementMarker(() ->{
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                })
                .waitSeconds(1)
                .strafeLeft(25)
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

