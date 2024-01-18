package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
@Config
public class ServoMicroSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private Servo MS1;
    private Servo MS2;
    private boolean isClosed=false;

    public ServoMicroSubsystem(RobotHardware robot){
        this.robot=robot;
        MS1= robot.MicroServo1;
        MS2= robot.MicroServo2;
    }
    public void setMicroServo1(double pos){
        MS1.setPosition(pos);
    }
    public void setMicroServo2(double pos){
        MS2.setPosition(pos);
    }

    public void setMicroServo12(){
        if(!isClosed) {
            MS1.setPosition(RobotHardware.MicroServoINCHIS1);
            MS2.setPosition(RobotHardware.MicroServoINCHIS2);
            isClosed=true;
        }
        else {
            MS1.setPosition(RobotHardware.MicroServoDESCHIS1);
            MS2.setPosition(RobotHardware.MicroServoDESCHIS2);
            isClosed=false;
        }
    }
}
