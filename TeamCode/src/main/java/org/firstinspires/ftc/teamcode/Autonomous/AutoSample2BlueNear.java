package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Stack;

import Teste.TestSleeveDetectionBlue;
import Teste.TestSleeveDetectionRed;

@Autonomous
public class AutoSample2BlueNear extends LinearOpMode {

    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotINIT;
    boolean isClosed=false;
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

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        //* While pt ca pivMotor sa ajunga la pozitie + detectie
        while(opModeInInit()){
            pivMotor.update();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
        }
        RobotHardware.State currentState;
        currentState= RobotHardware.State.TRAJECTORY_1;


        //* Traiectorii RoadRunner

        Pose2d myPose= new Pose2d(-61.2,11.5,0);
        //! pt fiecare caz, deplasarea pana la depunerea primului pixel
        //? LEFT
        TrajectorySequence traj1L= drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-38.175,9.125,Math.toRadians(90)))
                .waitSeconds(1)
                .addDisplacementMarker(()->{//functioneaza dupa 0.5 secunde
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .addDisplacementMarker(()->{//functioneaza duoa strafe(10)
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);//(new Pose2d(-43.75, 53.875, Math.toRadians()));
                })
                .build();
        TrajectorySequence traj2L= drive.trajectorySequenceBuilder(new Pose2d(-38.175,9.125,Math.toRadians(90)))
                .strafeRight(18.175)
                .lineToLinearHeading(new Pose2d(-40.25, 55, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMAX);
                })
                .addDisplacementMarker(() -> {
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                })
                .addDisplacementMarker(() -> {
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);
                })
                .build();
        //? CENTER
        TrajectorySequence traj1C= drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(25)
                .waitSeconds(3)
                .addTemporalMarker(0.5,()->{ //functioneaza dupa 0.5 secunde
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .addDisplacementMarker(()->{ //functioneaza duoa forward(10)
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                })
                .build();

        //? RIGHT
        TrajectorySequence traj1R= drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-38,11.5,Math.toRadians(-90)))
                .waitSeconds(3)
                .addTemporalMarker(0.5,()->{//functioneaza dupa 0.5 secunde
                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                })
                .addDisplacementMarker(()->{//functioneaza duoa strafe(10)
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                })

                .build();


        while(opModeIsActive()){
            switch (pipeline.getAnalysis()) {
                case LEFT:
                    switch (currentState)
                    {
                        case TRAJECTORY_1:
                            drive.followTrajectorySequence(traj1L);
                            currentState= RobotHardware.State.TRAJECTORY_2;
                            break;
                        case TRAJECTORY_2:
                            if(!drive.isBusy()){
                                drive.followTrajectorySequence(traj2L);
                            }
                    }
                    break;
                case CENTER:
                    switch (currentState)
                    {
                        case TRAJECTORY_1:
                            drive.followTrajectorySequence(traj1C);
                            currentState= RobotHardware.State.TRAJECTORY_2;
                            break;
                        case TRAJECTORY_2:
                            if(!drive.isBusy()){
                                drive.followTrajectorySequence(traj2L);//!schimba
                            }
                    }
                    break;

                case RIGHT:
                    switch (currentState)
                    {
                        case TRAJECTORY_1:
                            drive.followTrajectorySequence(traj1R);
                            currentState= RobotHardware.State.TRAJECTORY_2;
                            break;
                        case TRAJECTORY_2:
                            if(!drive.isBusy()){
                                drive.followTrajectorySequence(traj2L);//!schimba
                            }
                    }
                    break;
            }
            pivMotor.update();
            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.update();
            drive.update();
        }
    }
}
