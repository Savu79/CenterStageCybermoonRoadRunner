package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class PivotingMotorSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private DcMotorEx PM;

    private double power=0;

    public PivotingMotorSubsystem(RobotHardware robot){
        this.robot=robot;
        PM= robot.PivotingMotor;
    }
    public int getPivotingMotorPosition(){
        return PM.getCurrentPosition();
    }
    public void setPivotingPower(double power){
        PM.setPower(power);
    }
    public void manualMovement(double power)
    {
        robot.PivotingMotor.setPower(power);
    }
}
