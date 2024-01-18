package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group ="tele0p")
public class Tele0pPID extends LinearOpMode {
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    private PivotingMotorSubsystem pivMotor;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotMIN;
    boolean isClosed=false;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        extMotor= new ExtentionSubsystem(robot);
        pivMotor = new PivotingMotorSubsystem(robot);

        AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        waitForStart();

        while (opModeIsActive()) {
            pivMotor.update();
            extMotor.update();
            extTarget=extMotor.getExtentionPosition();
            //* MUTARE SERVOCONTROL LA POZITIE MAXIMA DUPA PivotMID
            if(pivMotor.getPivotingMotorPosition()> 200)
                AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

            if(pivMotor.getPivotingMotorPosition()< 200)
                AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

            //* MUTARE EXTENTION DE LA JOYSTICK
            if(gamepad2.left_stick_y!=0){
                extTarget+=(int)(gamepad2.left_stick_y*40);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            extMotor.setExtentionTarget(extTarget);

            if(gamepad2.right_stick_y!=0) {
                pivTarget+=(int)(gamepad2.right_stick_y*30);
                pivTarget= Range.clip(pivTarget, RobotHardware.PivotMIN, RobotHardware.PivotMAX);
            }
            pivMotor.setPivotingMotorTarget(pivTarget);

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

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            gamepad1.left_stick_x));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("pivotMotor", pivMotor.getPivotingMotorPosition());
            telemetry.addData("pivotMotorTarget", pivMotor.getPivotingMotorPosition());
            telemetry.addData("ExtentionMotor", extMotor.getExtentionPosition());
            telemetry.addData("AngControlServo", AngleControlServo.getPosition());
            telemetry.update();
        }
    }
}

