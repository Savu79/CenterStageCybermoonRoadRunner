package Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
@Config
@TeleOp(group ="test")
public class TestBratRidicarePID extends LinearOpMode {
    private RobotHardware robot= RobotHardware.getInstance();
    private PivotingMotorSubsystem pivMotor;
    int target=0;
    public void runOpMode(){

        robot.init(hardwareMap, telemetry);

        pivMotor= new PivotingMotorSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.b) target=250;
            if(gamepad1.y) target=1200;
            if(gamepad1.a) target=0;

            pivMotor.setPivotingMotorTarget(target);
            pivMotor.update();

            telemetry.addData("pozitie curenta: ", pivMotor.getPivotingMotorPosition());
            telemetry.addData("target: ", target);
            telemetry.update();

        }
    }
}
