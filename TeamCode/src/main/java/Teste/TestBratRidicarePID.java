package Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;

@TeleOp
public class TestBratRidicarePID extends LinearOpMode {
    private RobotHardware robot= RobotHardware.getInstance();
    private PivotingMotorSubsystem pivMotor;
    int error=0;
    int target=0;
    PIDController controller= new PIDController(RobotHardware.Ppivot, RobotHardware.Ipivot, RobotHardware.Dpivot);
    public void runOpMode(){
        waitForStart();
        if(opModeInInit()){
            robot.init(hardwareMap, telemetry);

            pivMotor= new PivotingMotorSubsystem(robot);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        }
        while(opModeIsActive()){
            controller.setPID(RobotHardware.Ppivot, RobotHardware.Ipivot, RobotHardware.Dpivot);

            if(gamepad1.a) target=500;
            if(gamepad1.b) target=900;
            if(gamepad1.x) target=0;

            error=target-pivMotor.getPivotingMotorPosition();

            //*robot.PivotingMotor.setPower(Range.clip(controller.calculate(0, error), -1, 1));

            pivMotor.setPivotingPower(Range.clip(controller.calculate(0, error), -1, 1));

        }
    }
}
