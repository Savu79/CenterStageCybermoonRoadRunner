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
@TeleOp
public class TestClimbPID extends LinearOpMode {
    private DcMotorEx climb1;
    private DcMotorEx climb2;
    private PIDController controller1;
    private PIDController controller2;

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


        }
    }
}
