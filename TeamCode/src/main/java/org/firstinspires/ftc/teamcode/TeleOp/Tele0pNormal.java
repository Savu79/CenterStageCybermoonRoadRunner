package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@Config
@TeleOp
public class Tele0pNormal extends LinearOpMode {
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    public DcMotorEx PivotingMotor;
    public DcMotorEx ExtentionMotor;
    private MecanumDrive drive;
    int extTarget=0;
    int pivTarget=RobotHardware.PivotMIN;
    boolean isClosed=false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    ),
                    gamepad1.left_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("pivotMotor", PivotingMotor.getCurrentPosition());
            telemetry.addData("pivotMotorTarget", PivotingMotor.getTargetPosition());
            telemetry.addData("ExtentionMotor", ExtentionMotor.getCurrentPosition());
            telemetry.addData("AngControlServo", AngleControlServo.getPosition());
            telemetry.update();
        }
    }
}

