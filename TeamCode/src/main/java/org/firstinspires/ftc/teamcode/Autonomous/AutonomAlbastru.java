package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
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

@Autonomous
public class AutonomAlbastru extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;
    OpenCvCamera backCamera;
    TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue pipeline;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() {

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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


        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack= hardwareMap.get(DcMotor.class, "leftBack");

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        int rightFrontTarget = rightFront.getCurrentPosition();
//        int rightBackTarget = rightBack.getCurrentPosition();
//        int leftFrontTarget = leftFront.getCurrentPosition();
//        int leftBackTarget = leftBack.getCurrentPosition();

        MicroServo1 = hardwareMap.get(Servo.class, "MicroServo1"); //!tine pixelul mov(pus pe foaie)
        MicroServo2 = hardwareMap.get(Servo.class, "MicroServo2"); //!tine pixelul galben(pus pe tabla)

        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean pusPixel = false;
        boolean parcat = false;

        waitForStart();
        if (opModeInInit()) {

            MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
            MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);

//           rightFrontTarget = rightFront.getCurrentPosition() + (int) (67 * DRIVE_COUNTS_PER_MM);
//           rightBackTarget = rightBack.getCurrentPosition() + (int) (67 * DRIVE_COUNTS_PER_MM);
//           leftFrontTarget = leftFront.getCurrentPosition() + (int) (67 * DRIVE_COUNTS_PER_MM);
//           leftBackTarget = leftBack.getCurrentPosition() + (int) (67 * DRIVE_COUNTS_PER_MM);

//           rightFront.setTargetPosition(rightFrontTarget);
//           rightBack.setTargetPosition(rightBackTarget);
//           leftFront.setTargetPosition(leftFrontTarget);
//           leftBack.setTargetPosition(leftBackTarget);

            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
            leftBack.setPower(0.5);
            leftFront.setPower(0.5);
        }
            while (opModeIsActive() && (rightFront.isBusy() || rightBack.isBusy() || leftFront.isBusy() || leftBack.isBusy()) ) {

                sleep(2000);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);

                if (!pusPixel) {
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
                    pusPixel = true;
                }
                if (!parcat) {
                    //inapoi de unde am inceput
                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);
                    sleep(2000);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    //strafing spre stanga pentru parcare
                    rightFront.setPower(0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(-0.5);
                    sleep(2000);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    parcat = true;
                }
            }
        }
    }