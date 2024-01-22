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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Teste.TestSleeveDetectionBlue;

@Autonomous
public class AutonomAlbastruFar extends LinearOpMode {

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
    TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue pipeline;

    @Override
    public void runOpMode() {

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
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        MicroServo1 = hardwareMap.get(Servo.class, "MicroServo1");//albastru, tine pixelul mov(pus pe foita)
        MicroServo2 = hardwareMap.get(Servo.class, "MicroServo2");//negru, tine pixelul galben(pus pe tabla)
        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo = hardwareMap.get(Servo.class, "ControlServo");//albastru+negru
        AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

        ServoAvion = hardwareMap.get(Servo.class, "AvionServo");
        ServoAvion.setPosition(RobotHardware.AvionParcat);


        PivotingMotor = hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        PivotingMotor.setPower(0.75);
        PivotingMotor.setTargetPosition(0);
        PivotingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        ExtentionMotor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setPower(0.75);
        ExtentionMotor.setTargetPosition(0);
        ExtentionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (opModeInInit()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
            MicroServo1.setPosition(RobotHardware.MicroServoINCHIS2);
        }
        if (opModeIsActive()) {
            switch (pipeline.getAnalysis()) {
                case LEFT:
                    //rotire stanga
                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);

                    break;

                case CENTER:
                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);

                    sleep(100);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    //*incepem
                    /*sleep(250);
                    PivotingMotor.setTargetPosition(RobotHardware.PivotMID);
                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    sleep(250);


                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    sleep(250);

                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);//rotire dreapta

                    sleep(1000);

                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    sleep(250);

                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);//rotire stanga, e cu spatele la tabla

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    sleep(250);

                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);

                    sleep(1000);

                    PivotingMotor.setTargetPosition(RobotHardware.PivotMAX);
                    sleep(250);
                    MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                    sleep(250);
                    PivotingMotor.setTargetPosition(RobotHardware.PivotMID);

                    rightFront.setPower(0.5);//strafe left, langa tabla
                    rightBack.setPower(-0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(-0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);

                    sleep(250);

                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);

                    sleep(250);

*/

                    break;

                case RIGHT:
                    //rotire dreapta
                    rightFront.setPower(-0.5);
                    rightBack.setPower(-0.5);
                    leftBack.setPower(0.5);
                    leftFront.setPower(0.5);

                    sleep(1000);

                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);

                    break;
            }
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
            leftBack.setPower(-0.5);
            leftFront.setPower(-0.5);

            sleep(500);

            rightFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);
            leftFront.setPower(0);
        }
    }
}