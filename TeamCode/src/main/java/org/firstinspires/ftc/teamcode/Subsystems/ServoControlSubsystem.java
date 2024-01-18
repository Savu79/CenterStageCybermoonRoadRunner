package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
@Config
public class ServoControlSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private Servo servo;
    public ServoControlSubsystem(RobotHardware robot){
        this.robot= robot;
        servo=robot.AngleControlServo;
    }
    public void setPositionServoControl(double pos){
        servo.setPosition(pos);
    }
    public void setPositionServoControl1(double pos){
        servo.setPosition(pos);
    }
    public void setAnglePositionServoControl(int angle){
        double pos=angle/300.00;
        servo.setPosition(pos);
    }
}
