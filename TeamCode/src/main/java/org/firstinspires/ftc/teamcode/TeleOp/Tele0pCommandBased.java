package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ServoControlSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ServoMicroSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "Tele0pCommandBased")
public class Tele0pCommandBased extends CommandOpMode {
    private ElapsedTime timer;
    private RobotHardware robot = RobotHardware.getInstance();
    private ServoMicroSubsystem servoMicro;
    private ServoControlSubsystem servoControl;
    private PivotingMotorSubsystem pivotingMotorSubsystem;
    private ExtentionSubsystem extentionSubsystem;
    private SampleMecanumDrive drive;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    int extTarget=0;
    int pivTarget=0;
    //OpenCvCamera backCamera;
    //SleeveDetection.SkystoneDeterminationPipeline pipeline;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        servoMicro = new ServoMicroSubsystem(robot);
        servoControl = new ServoControlSubsystem(robot);
        pivotingMotorSubsystem = new PivotingMotorSubsystem(robot);
        extentionSubsystem = new ExtentionSubsystem(robot);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //*DESCHIDERE/INCHIDERE AMBELE SERVOURI
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    sleep(800);
                    servoMicro.setMicroServo12();
                });

        //*SETARE EXTENTION SUS
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new InstantCommand(() -> pivotingMotorSubsystem.setPivotingMotorTarget(RobotHardware.PivotMAX))));

        //*SETARE EXTENTION MID
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> pivotingMotorSubsystem.setPivotingMotorTarget(RobotHardware.PivotMID));

        //*SETARE EXTENTION JOS
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> pivotingMotorSubsystem.setPivotingMotorTarget(RobotHardware.PivotMIN));


        //*DECLARATII CAMERA
        /*backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new SleeveDetection.SkystoneDeterminationPipeline();
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
        });*/
    }

    @Override
    public void run() {
        super.run();

        extTarget=extentionSubsystem.getExtentionCurrentPosition();
        //robot.read(intake);
        //robot.loop(intake);
        //robot.write(intake);
        extTarget=robot.ExtentionMotor.getCurrentPosition();
        //* MUTARE SERVOCONTROL LA POZITIE MAXIMA DUPA PivotMID
        if(robot.PivotingMotor.getCurrentPosition()> RobotHardware.PivotMID)
            robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

        if(robot.PivotingMotor.getCurrentPosition()< RobotHardware.PivotMID && robot.PivotingMotor.getCurrentPosition()>200)
            robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        //* MUTARE EXTENTION DE LA JOYSTICK
        if(gamepad2.left_stick_y!=0){
            extTarget+=(int)(gamepad2.left_stick_y*40);
            extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
        }
        robot.ExtentionMotor.setTargetPosition(extTarget);

        if(gamepad2.right_stick_y!=0) {
            pivTarget+=(int)(gamepad2.right_stick_y*30);
            pivTarget= Range.clip(pivTarget, RobotHardware.PivotMIN, RobotHardware.PivotMAX);
        }
        robot.PivotingMotor.setTargetPosition(pivTarget);

        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x));

        drive.updatePoseEstimate();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}
