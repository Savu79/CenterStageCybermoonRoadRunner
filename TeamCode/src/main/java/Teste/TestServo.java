package Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group ="test")
public class TestServo extends LinearOpMode
{
    Servo servo1; //dreptul
    Servo servo2; //stangul
    double pozitie1 = 1;
    double pozitie2 = 0;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();
        while(opModeIsActive()) {


            if (gamepad1.right_stick_y != 0){
                pozitie1 += 0.0005 * gamepad1.right_stick_y;
            }
            if (gamepad1.left_stick_y != 0){
                pozitie2 += 0.0005 * gamepad1.left_stick_y;
            }


            pozitie1=Math.max(0,Math.min(1,pozitie1));
            pozitie2=Math.max(0,Math.min(1,pozitie2));

            if(gamepad1.a)
            {
                pozitie1=0;
                pozitie2=0;
            }
            servo1.setPosition(pozitie1);
            servo2.setPosition(pozitie2);
            telemetry.addData("pozitie1: ", servo1.getPosition());
            telemetry.addData("pozitie2: ", servo2.getPosition());
            telemetry.update();

        }
    }
}

