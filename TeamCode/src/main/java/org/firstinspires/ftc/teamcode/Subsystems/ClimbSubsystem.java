package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ClimbSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private DcMotorEx climb1;
    private DcMotorEx climb2;
    private PIDController controller1;
    private PIDController controller2;

    private int target=0;
    public static double P = 0.003;
    public static double I = 0.0;
    public static double D = 0.00051;


    public ClimbSubsystem(RobotHardware robot){
        this.robot= robot;
        climb1= robot.ClimbRight;
        climb2= robot.ClimbLeft;
        controller1= new PIDController(P,I,D);
        controller2= new PIDController(P,I,D);
    }
    public void update(){
        this.controller1.setPID(P, I, D);
        this.controller2.setPID(P, I, D);

        int error1=target-getExtentionPosition1();
        int error2=target-getExtentionPosition2();

        climb1.setPower(Range.clip(controller1.calculate(0, error1), -1, 1));
        climb2.setPower(Range.clip(controller2.calculate(0,error2), -1, 1));
    }

    public int getExtentionPosition1(){
        return climb1.getCurrentPosition();
    }
    public int getExtentionPosition2(){
        return climb2.getCurrentPosition();
    }
    public void setExtentionPower1(double power){
        climb1.setPower(power);
    }
    public void setExtentionPower2(double power){
        climb2.setPower(power);
    }
    public void setExtentionTarget(int target){
        this.target=target;
    }

}
