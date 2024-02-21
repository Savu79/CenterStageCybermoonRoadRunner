package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
@Config
public class ExtentionSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private DcMotorEx EM;
    private PIDController controller;

    private int target=0;
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.00051;


    public ExtentionSubsystem(RobotHardware robot){
        this.robot= robot;
        EM= robot.ExtentionMotor;
        controller= new PIDController(P,I,D);
    }
    public void update(){
        this.controller.setPID(P, I, D);
        int error=target-getExtentionPosition();
        EM.setPower(Range.clip(controller.calculate(0, error), -0.5, 0.5));
    }

    public int getExtentionPosition(){
        return EM.getCurrentPosition();
    }
    public double getPower(){
        return EM.getPower();
    }
    public void setExtentionPower(double power){
        EM.setPower(power);
    }
    public void setExtentionTarget(int target){
        this.target=target;
    }

}

