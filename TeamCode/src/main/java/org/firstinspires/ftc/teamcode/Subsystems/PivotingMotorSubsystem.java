package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
@Config
public class PivotingMotorSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private DcMotorEx PM;
    private PIDController controller;
    private int target=RobotHardware.PivotMID;

    public static double P=0.004;
    public static double I=0;
    public static double D=0.01;

    public PivotingMotorSubsystem(RobotHardware robot){
        this.robot=robot;
        PM= robot.PivotingMotor;
        controller= new PIDController(P,I,D);
    }
    public void update(){
        controller.setPID(P,I,D);
        int error=target-getPivotingMotorPosition();
        PM.setPower(Range.clip(controller.calculate(0,error),-1,1));
    }
    public int getPivotingMotorPosition(){
        return PM.getCurrentPosition();
    }
    public void setPivotingMotorPower(double power){
        PM.setPower(power);
    }
    public void setPivotingMotorTarget(int target){
        this.target=target;
    }

}
