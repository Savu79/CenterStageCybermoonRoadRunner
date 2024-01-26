package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
@Config
public class RobotHardware {

    /*public static int pipelineStage = 0;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 0;
    public static double HUE_MAX = 90;
    public static double SATURATION_MIN = 150;
    public static double SATURATION_MAX = 255;
    public static double VALUE_MIN = 150;
    public static double VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 2500;
    public static String BLUR = "Box Blur";*/
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;
    public Servo ServoAvion;

    public DcMotorEx PivotingMotor;
    public DcMotorEx ExtentionMotor;

    private HardwareMap hardwareMap;

    //public SleeveDetection.SkystoneDeterminationPipeline pipeline;
    //public OpenCvCamera backCamera;
    private static RobotHardware instance = null;

    public boolean enabled;

    //! VALORI CONSTANTE

    public static double MicroServoDESCHIS1=0.85;
    public static double MicroServoDESCHIS2=0.35;
    public static double MicroServoINCHIS1=0.50;
    public static double MicroServoINCHIS2=0.71;

    public static int PivotMAX=1279;
    public static int PivotMID=250;
    public static int PivotMIN=0;
    public static int PivotINIT=533;//? de testat

    public static int ExtentionMAX=900;
    public static int ExtentionMIN=0;

    public static double ServoControlMAX=0.55;
    public static double ServoControlMID=0.2;
    public static double ServoControlMIN=0.0315;

    public static double AvionParcat=1;
    public static double AvionDecolare=0.75;
    public enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4// First, follow a splineTo() trajectory
    }

    //public static double ServoControlMAX=0.75;
    //public static double ServoControlMID=0.9;
    //public static double ServoControlMIN=1;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        //TODO declaram motoare
        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");//albastru
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");//negru
        MicroServo1.setPosition(RobotHardware.MicroServoDESCHIS1);
        MicroServo2.setPosition(RobotHardware.MicroServoDESCHIS2);

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");//albastru+negru
        AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

        ServoAvion=hardwareMap.get(Servo.class, "AvionServo");
        ServoAvion.setPosition(AvionParcat);


        PivotingMotor= hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        PivotingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setPower(0.75);
        ExtentionMotor.setTargetPosition(0);
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        pipeline = new SleeveDetection.SkystoneDeterminationPipeline();
//        backCamera.setPipeline(pipeline);

    }

    public void loop(ExtentionSubsystem extention) {

    }

    public void read() {
//        try {
//            intake.read();
//        } catch (Exception ignored) {
    }

    public void write() {
//            try {
//                intake.write();
//            } catch (Exception ignored){}
    }
}
