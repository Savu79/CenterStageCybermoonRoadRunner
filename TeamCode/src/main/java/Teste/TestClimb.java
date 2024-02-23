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
public class TestClimb extends LinearOpMode {
    private DcMotorEx climb1;
    private DcMotorEx climb2;


    public void runOpMode(){
        climb1=hardwareMap.get(DcMotorEx.class, "ClimbRight");
        climb2=hardwareMap.get(DcMotorEx.class, "ClimbLeft");
        climb2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){

            climb1.setPower(gamepad1.right_stick_y);
            climb2.setPower(gamepad1.left_stick_y);
            telemetry.addData("climb1", climb1.getCurrentPosition());
            telemetry.addData("climb2", climb2.getCurrentPosition());
            telemetry.update();
            if(gamepad1.right_stick_button) {
                climb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                climb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


        }
    }
}
