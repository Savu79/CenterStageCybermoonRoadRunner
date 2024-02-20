package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
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
public class Tele0pPIDNEW extends LinearOpMode {
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
        //extMotor= new ExtentionSubsystem(robot);
        pivMotor = new PivotingMotorSubsystem(robot);

        pivMotor.setPivotingMotorTarget(RobotHardware.PivotMIN);
        //extMotor.setExtentionTarget(0);
        //robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        waitForStart();

        while (opModeIsActive()) {
            pivMotor.update();
            //* MUTARE SERVOCONTROL LA POZITIE MAXIMA DUPA PivotMID
            if(pivMotor.getPivotingMotorPosition()> 200)
                robot.AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

            if(pivMotor.getPivotingMotorPosition()< 200 && extTarget>400)
                robot.AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

            if(pivMotor.getPivotingMotorPosition()< 200 && extTarget<400)
                robot.AngleControlServo.setPosition(RobotHardware.ServoControlMID);

            //* MUTARE EXTENTION
            if(gamepad2.left_stick_y!=0){
                extTarget+=(int)(gamepad2.left_stick_y*40);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            if(gamepad2.a) extTarget= RobotHardware.ExtentionMIN;
            if(gamepad2.b) extTarget= RobotHardware.ExtentionMAX;

            robot.ExtentionMotor.setTargetPosition(extTarget);

            //*MUTARE PIVOTING
            if(gamepad2.right_stick_y!=0) {
                pivTarget+=(int)(gamepad2.right_stick_y*30);
                pivTarget= Range.clip(pivTarget, RobotHardware.PivotMIN, RobotHardware.PivotMAX);
            }
            pivMotor.setPivotingMotorTarget(pivTarget);

            if(gamepad2.dpad_up)
            {
                pivTarget=RobotHardware.PivotMAX;
                extTarget=RobotHardware.ExtentionMIN;
            }
            if(gamepad2.dpad_right)
            {
                pivTarget=RobotHardware.PivotMIN;
                extTarget=RobotHardware.ExtentionMIN;
            }
            if(gamepad2.dpad_down)
            {
                pivTarget=RobotHardware.PivotMIN;
                extTarget=RobotHardware.ExtentionMAX;

            }

            //*MUTARE POZTII MICROCSERVO
            if(gamepad1.a)
            {
                if(!isClosed) {
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);
                    sleep(700);
                    isClosed=true;
                }
                else {
                    robot.MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                    robot.MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);
                    isClosed=false;
                    sleep(700);
                }
            }
            //*AVION
            if(gamepad1.dpad_left)
            {
                robot.ServoAvion.setPosition(RobotHardware.AvionDecolare);
            }
            if(gamepad1.dpad_up)
            {
                robot.ServoAvion.setPosition(RobotHardware.AvionParcat);
            }
            //*DRIVE
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("pivotMotor", pivMotor.getPivotingMotorPosition());
            telemetry.addData("pivotMotorTarget", pivMotor.getPivotingMotorPosition());
            telemetry.addData("ExtentionMotor", robot.ExtentionMotor.getCurrentPosition());
            telemetry.addData("AngControlServo", robot.AngleControlServo.getPosition());
            telemetry.update();
        }
    }
}


