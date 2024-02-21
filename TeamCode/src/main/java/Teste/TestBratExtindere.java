package Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group ="test")
public class TestBratExtindere extends LinearOpMode {

    DcMotor extindatoarea;
    //Gamepad gamepad1=new Gamepad();
    @Override
    public void runOpMode() {


        extindatoarea = hardwareMap.get(DcMotor.class, "ExtensionMotor");
        extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.right_stick_button) extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("pozitie: ",extindatoarea.getCurrentPosition());
            telemetry.update();
            extindatoarea.setPower(gamepad1.right_stick_y);
        }
    }
}
