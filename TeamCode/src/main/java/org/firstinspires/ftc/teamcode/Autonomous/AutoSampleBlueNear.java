package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Teste.TestSleeveDetectionRed;

public class AutoSampleBlueNear extends LinearOpMode {

    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotINIT;
    boolean isClosed=false;
    OpenCvCamera backCamera;
    TestSleeveDetectionRed.SkystoneDeterminationPipelineRed pipeline;
    public void runOpMode(){
        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        pivMotor = new PivotingMotorSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new TestSleeveDetectionRed.SkystoneDeterminationPipelineRed();
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

        pivMotor.setPivotingMotorTarget(RobotHardware.PivotINIT); //! PivINIT are valaoarea 0

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

        //* Traiectorii RoadRunner

        Pose2d myPose= new Pose2d(-61.2,11.5,0);
        //! pt fiecare caz, deplasarea pana la depunerea primului pixel
        //? LEFT
        TrajectorySequence traj1L= drive.trajectorySequenceBuilder(myPose)
                                .lineToLinearHeading(new Pose2d(-38,11.5,Math.toRadians(90)))
                                .waitSeconds(3)
                                .addTemporalMarker(0.5,()->{//functioneaza dupa 0.5 secunde
                                    pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
                                    robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);
                                })
                                .addDisplacementMarker(()->{//functioneaza duoa strafe(10)
                                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                                })
                                .build();
        //? CENTER
        TrajectorySequence traj1C= drive.trajectorySequenceBuilder(myPose)
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
        TrajectorySequence traj1R= drive.trajectorySequenceBuilder(myPose)
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


        waitForStart(); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        switch(pipeline.getAnalysis()){
            case LEFT:
                drive.followTrajectorySequenceAsync(traj1L);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(traj1R);
                break;
            case CENTER:
                drive.followTrajectorySequenceAsync(traj1C);
                break;
        }

        while(opModeIsActive()){
            pivMotor.update();
            myPose=drive.getPoseEstimate();
            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.update();
        }
    }
}
