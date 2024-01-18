package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@Autonomous
public class Autonom extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack= hardwareMap.get(DcMotor.class, "leftBack");

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int rightFrontTarget = rightFront.getCurrentPosition();
        int rightBackTarget = rightBack.getCurrentPosition();
        int leftFrontTarget = leftFront.getCurrentPosition();
        int leftBackTarget = leftBack.getCurrentPosition();

        MicroServo1 = hardwareMap.get(Servo.class, "MicroServo1"); //!tine pixelul mov(pus pe foaie)
        MicroServo2 = hardwareMap.get(Servo.class, "MicroServo2"); //!tine pixelul galben(pus pe tabla)

        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        AngleControlServo.setPosition(RobotHardware.ServoControlMIN);

        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int pozitiePahar = 2; // valoare de test
        boolean pusPixel = false;

        waitForStart();
        if (opModeInInit()) {
            // Create target positions

            MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
            MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);

            //dist de unde e robotul la cele trei foi (in teorie)
            rightFrontTarget = rightFront.getCurrentPosition() + (int)(67 * DRIVE_COUNTS_PER_MM);
            rightBackTarget = rightFront.getCurrentPosition() + (int)(67 * DRIVE_COUNTS_PER_MM);
            leftFrontTarget = rightFront.getCurrentPosition() + (int)(67 * DRIVE_COUNTS_PER_MM);
            leftBackTarget = rightFront.getCurrentPosition() + (int)(67 * DRIVE_COUNTS_PER_MM);

            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);
            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);

            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
            leftBack.setPower(0.5);
            leftFront.setPower(0.5);

            pozitiePahar = 2; //se afla pozitia paharului

            while (opModeIsActive() && (rightFront.isBusy() || rightBack.isBusy() || leftFront.isBusy() || leftBack.isBusy()) ) {

                sleep(2000);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);

                if (!pusPixel) {
                    switch (pozitiePahar) {
                        case 1: //!paharul la stanga
                            //rotire spre stanga
                            MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                            break;

                        case 2: //!paharul pe mijloc
                            //dat drumu la pixel direct
                            MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                            break;

                        case 3: //!pahrul pe dreapta
                            //rotire spre dreapta
                            MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
                            break;
                    }
                    pusPixel = true;
                }
            }
        }
    }
}