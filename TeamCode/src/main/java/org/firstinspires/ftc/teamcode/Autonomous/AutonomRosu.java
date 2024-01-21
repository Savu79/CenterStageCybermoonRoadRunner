package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import Teste.TestSleeveDetectionBlue;
import Teste.TestSleeveDetectionRed;

@Autonomous
public class AutonomRosu extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private DcMotorEx PivotingMotor;
    private DcMotorEx ExtentionMotor;

    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;
    public Servo ServoAvion;

    OpenCvCamera backCamera;
    TestSleeveDetectionRed.SkystoneDeterminationPipelineRed pipeline;

    @Override
    public void runOpMode() {

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");//albastru
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");//negru
        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");//albastru+negru
        AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

        ServoAvion=hardwareMap.get(Servo.class, "AvionServo");
        ServoAvion.setPosition(RobotHardware.AvionParcat);


        PivotingMotor= hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        PivotingMotor.setPower(0.75);
        PivotingMotor.setTargetPosition(0);
        PivotingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setPower(0.75);
        ExtentionMotor.setTargetPosition(0);
        ExtentionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (opModeInInit())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
        }
        switch (pipeline.getAnalysis()) {
            case LEFT:
                //rotire spre stanga
                rightFront.setPower(0.5);
                rightBack.setPower(0.5);
                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                sleep(500);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);

                MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);

                //rotire la directia initiala
                rightFront.setPower(-0.5);
                rightBack.setPower(-0.5);
                leftBack.setPower(0.5);
                leftFront.setPower(0.5);
                sleep(500);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);
                break;

            case CENTER:
                MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                break;

            case RIGHT:
                //rotire spre dreapta
                rightFront.setPower(-0.5);
                rightBack.setPower(-0.5);
                leftBack.setPower(0.5);
                leftFront.setPower(0.5);
                sleep(500);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);

                MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);

                //rotire la directia initiala
                rightFront.setPower(0.5);
                rightBack.setPower(0.5);
                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                sleep(500);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);
                break;
        }
    }
}