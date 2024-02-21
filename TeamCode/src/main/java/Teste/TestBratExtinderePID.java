package Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
@Config
@TeleOp(group ="test")
public class TestBratExtinderePID extends LinearOpMode {
    private RobotHardware robot= RobotHardware.getInstance();
    private ExtentionSubsystem extMotor;
    int target=0;
    public void runOpMode(){

        robot.init(hardwareMap, telemetry);

        extMotor= new ExtentionSubsystem(robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.b) target=750;
            if(gamepad1.y) target=900;
            if(gamepad1.a) target=0;

            extMotor.setExtentionTarget(target);
            extMotor.update();

            telemetry.addData("pozitie curenta: ", extMotor.getExtentionPosition());
            telemetry.addData("target: ", target);
            telemetry.addData("Power", extMotor.getPower());
            telemetry.update();

        }
    }
}
