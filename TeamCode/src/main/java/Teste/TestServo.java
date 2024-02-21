package Teste;



import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestServo extends LinearOpMode
{
    Servo MicroServo1; //dreptul
    Servo MicroServo2; //dreptul
    Servo AngleControlServo; //stangul
    double pozitie1 = RobotHardware.MicroServoINCHIS1;
    double pozitie2 = RobotHardware.MicroServoINCHIS2;
    double pozitieC= RobotHardware.ServoControlMIN;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");//albastru
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");
        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        waitForStart();
        while(opModeIsActive()) {
            pozitie1 += 0.0005 * gamepad1.right_stick_y;
            pozitie2 += 0.0005 * gamepad1.left_stick_y;
            pozitieC += 0.0005 * gamepad2.right_stick_y;

            pozitie1= Range.clip(pozitie1, -1, 1);
            pozitie2= Range.clip(pozitie2, -1, 1);
            pozitieC= Range.clip(pozitieC, -1, 1);

            MicroServo1.setPosition(pozitie1);
            MicroServo2.setPosition(pozitie2);
            AngleControlServo.setPosition(pozitieC);
            telemetry.addData("pozitie1: ", MicroServo1.getPosition());
            telemetry.addData("pozitie2: ", MicroServo2.getPosition());
            telemetry.addData("pozitie Control", AngleControlServo.getPosition());
            telemetry.update();

        }
    }
}

