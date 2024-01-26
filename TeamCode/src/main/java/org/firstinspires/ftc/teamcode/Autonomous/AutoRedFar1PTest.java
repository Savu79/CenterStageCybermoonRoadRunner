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
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Teste.TestSleeveDetectionBlue;

@Autonomous
public class AutoRedFar1PTest extends LinearOpMode {
    //*1P de la un pixel
    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    private TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue.SkystonePosition pi;
    int extTarget=0;
    Pose2d myPose;
    int pivTarget=RobotHardware.PivotINIT;
    boolean isClosed=false;
    boolean afost=false;
    OpenCvCamera backCamera;
    TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue pipeline;
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void runOpMode(){
        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        pivMotor = new PivotingMotorSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue();
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

        drive.setPoseEstimate(new Pose2d(10, 61, Math.toRadians(270)));

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
        /*
        TrajectorySequence traj1L= drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(10, 34, Math.toRadians(360)))
                .strafeRight(25)
                .lineTo(new Vector2d(25, 9))
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)))
                .strafeLeft(28)
                .lineTo(new Vector2d(61, 10))
                .waitSeconds(1)
                .build();
         */

        TrajectorySequence traj1L =  drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
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
                .build();

        TrajectorySequence traj1C =  drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                .lineTo(new Vector2d(-35, -38))
                .waitSeconds(1)
                .strafeLeft(20)
                .lineTo(new Vector2d(-52, -9))
                .lineToLinearHeading(new Pose2d(25, -9,Math.toRadians(180)))
                .lineTo(new Vector2d(47, -34))
                .waitSeconds(1)
                .strafeRight(24)
                .lineTo(new Vector2d(59, -10))
                .waitSeconds(1)
                .build();

        TrajectorySequence traj1R =  drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                .lineTo(new Vector2d(-40, -34))
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .strafeLeft(25)
                .lineTo(new Vector2d(25, -9))
                .lineToLinearHeading(new Pose2d(47, -42, Math.toRadians(180)))
                .waitSeconds(1)
                .strafeRight(32)
                .lineTo(new Vector2d(59, -10))
                .waitSeconds(1)
                .build();


        while(opModeIsActive()){
            drive.update();
            switch (pi) {
                case LEFT:
                    if(!afost) {
                        drive.followTrajectorySequence(traj1L);
                        afost=true;
                    }
                    break;
                case CENTER:
                    if(!afost) {
                        drive.followTrajectorySequence(traj1C);
                        afost=true;
                    }

                    break;

                case RIGHT:
                    if(!afost) {
                        drive.followTrajectorySequence(traj1R);
                        afost=true;
                    }

                    break;
            }
            //pivMotor.update();
            myPose=drive.getPoseEstimate();
            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.update();
        }
    }
}
