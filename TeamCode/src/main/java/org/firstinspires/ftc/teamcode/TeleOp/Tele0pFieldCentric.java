package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group ="tele0p")
public class Tele0pFieldCentric extends LinearOpMode {
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    public DcMotorEx PivotingMotor;
    public DcMotorEx ExtentionMotor;
    private SampleMecanumDrive drive;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotMIN;
    boolean isClosed=false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        PivotingMotor= hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        PivotingMotor.setTargetPosition(RobotHardware.PivotMIN);
        PivotingMotor.setPower(1);
        PivotingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setTargetPosition(RobotHardware.ExtentionMIN);
        ExtentionMotor.setPower(1);
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        waitForStart();

        while (opModeIsActive()) {
            extTarget=ExtentionMotor.getCurrentPosition();
            //* MUTARE SERVOCONTROL LA POZITIE MAXIMA DUPA PivotMID
            if(PivotingMotor.getCurrentPosition()> 200)
                AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

            if(PivotingMotor.getCurrentPosition()< 200)
                AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

            //* MUTARE EXTENTION DE LA JOYSTICK
            if(gamepad2.left_stick_y!=0){
                extTarget+=(int)(gamepad2.left_stick_y*40);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            ExtentionMotor.setTargetPosition(extTarget);

            if(gamepad2.right_stick_y!=0) {
                pivTarget+=(int)(gamepad2.right_stick_y*30);
                pivTarget= Range.clip(pivTarget, RobotHardware.PivotMIN, RobotHardware.PivotMAX);
            }
            PivotingMotor.setTargetPosition(pivTarget);

            if(gamepad2.dpad_up)
            {
                pivTarget=RobotHardware.PivotMAX;
            }
            if(gamepad2.dpad_right)
            {
                pivTarget=RobotHardware.PivotMID;
            }
            if(gamepad2.dpad_down)
            {
                pivTarget=RobotHardware.PivotMIN;
            }
            if(gamepad1.a)
            {
                if(!isClosed) {
                    MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
                    MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
                    sleep(700);
                    isClosed=true;
                }
                else {
                    MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                    isClosed=false;
                    sleep(700);
                }
            }

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
            com.acmerobotics.roadrunner.geometry.Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("pivotMotor", PivotingMotor.getCurrentPosition());
            telemetry.addData("pivotMotorTarget", PivotingMotor.getTargetPosition());
            telemetry.addData("ExtentionMotor", ExtentionMotor.getCurrentPosition());
            telemetry.addData("AngControlServo", AngleControlServo.getPosition());
            telemetry.update();
        }
    }
}

