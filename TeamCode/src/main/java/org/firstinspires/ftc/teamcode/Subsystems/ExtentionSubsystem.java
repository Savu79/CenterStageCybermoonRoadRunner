package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ExtentionSubsystem extends SubsystemBase {
    private RobotHardware robot;

    public static double P = 0.0121;
    public static double I = 0.0;
    public static double D = 0.00051;
    private double power=0;
    private int currentPosition=0;
    private int targetPosition=0;
    private PIDController controller= new PIDController(P,I,D);

    public ExtentionSubsystem(RobotHardware robot){
        this.robot= robot;
    }
    public void write(){
        robot.ExtentionMotor.setPower(power);
    }

    public void loop(){
        this.controller.setPID(P, I, D);
        power = Range.clip((-controller.calculate(currentPosition, targetPosition)), -1, 1);

    }
    public void read(){
        currentPosition=robot.ExtentionMotor.getCurrentPosition();
    }


    public void setExtentionTarget(int targetPosition){
        this.targetPosition=targetPosition;
        robot.ExtentionMotor.setTargetPosition(targetPosition);
    }
    public int getExtentionTarget(){
        return this.targetPosition;
    }
    public int getExtentionCurrentPosition(){
        return this.currentPosition;
    }
}
