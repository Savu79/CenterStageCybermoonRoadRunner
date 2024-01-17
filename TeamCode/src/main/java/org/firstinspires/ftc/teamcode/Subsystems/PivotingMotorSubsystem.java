package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class PivotingMotorSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private DcMotorEx PM;
    private PIDController controller;
    private double power=0;
    private int target=0;

    public PivotingMotorSubsystem(RobotHardware robot){
        this.robot=robot;
        PM= robot.PivotingMotor;
        controller= new PIDController(RobotHardware.Ppivot, RobotHardware.Ipivot, RobotHardware.Dpivot);
    }
    public void update(){
        controller.setPID(RobotHardware.Ppivot, RobotHardware.Ipivot, RobotHardware.Dpivot);
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
    public void manualMovement(double power)
    {
        robot.PivotingMotor.setPower(power);
    }
}
