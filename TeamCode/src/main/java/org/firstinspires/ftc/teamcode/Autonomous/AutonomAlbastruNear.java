package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Teste.TestSleeveDetectionBlue;

@Autonomous
public class AutonomAlbastruNear extends LinearOpMode {

    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotMID+150;
    private static ElapsedTime timer1 = new ElapsedTime();
    OpenCvCamera backCamera;
    TestSleeveDetectionBlue.SkystoneDeterminationPipelineBlue pipeline;

    int pozitiePahar = 0;


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

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        //extMotor= new ExtentionSubsystem(robot);
        pivMotor = new PivotingMotorSubsystem(robot);

        pivMotor.setPivotingMotorTarget(RobotHardware.PivotMID);

        while (opModeInInit())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            timer1.reset();
            timer1.startTime();
            switch (pipeline.getAnalysis()) {
                case LEFT:
                    pozitiePahar = 1;
                    break;
                case CENTER:
                    pozitiePahar = 2;
                    break;

                case RIGHT:
                    pozitiePahar = 3;
                    break;
            }
        }
        while (opModeIsActive()) {
            pivMotor.update();

            switch (pozitiePahar) {
                case 1:
                    if (timer1.milliseconds() < 1000) {
                        drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    }
                    else if (timer1.milliseconds() > 1000 && timer1.milliseconds() < 1050) {
                        drive.setMotorPowers(0, 0, 0, 0);
                    }
                    else if (timer1.milliseconds() > 1050 && timer1.milliseconds() < 1300) {
                        drive.setMotorPowers(-0.5, -0.5, 0.5, 0.5);
                    }

                    break;

                case 2:

                    break;

                case 3:

                    break;
            }
        }
    }
}