package Teste;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(group ="test")
public class TestClimbPID extends LinearOpMode {
    private DcMotorEx climb1;
    private DcMotorEx climb2;
    private PIDController controller1 = new PIDController(P, I, D);
    private PIDController controller2 = new PIDController(P, I, D);

    public static int target=0;
    public static double P = 0.003;
    public static double I = 0.0;
    public static double D = 0.00051;
    public void runOpMode(){
        climb1=hardwareMap.get(DcMotorEx.class, "ClimbRight");
        climb2=hardwareMap.get(DcMotorEx.class, "ClimbLeft");
        climb2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) target=100;
            if(gamepad1.x) target=800;
            if(gamepad1.b) target=1000;
            if(gamepad1.y) target=1650;
            this.controller1.setPID(P, I, D);
            this.controller2.setPID(P, I, D);

            int error1=target-climb1.getCurrentPosition();
            int error2=target-climb2.getCurrentPosition();

            climb1.setPower(Range.clip(controller1.calculate(0, error1), -1, 1));
            climb2.setPower(Range.clip(controller2.calculate(0,error2), -1, 1));

            if(gamepad1.right_stick_button) {
                climb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                climb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.addData("pozitie reala1: ", climb1.getCurrentPosition());
            telemetry.addData("pozitie reala2: ", climb2.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();

        }
    }
}
