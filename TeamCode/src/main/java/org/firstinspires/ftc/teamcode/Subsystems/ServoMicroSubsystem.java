package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ServoMicroSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private boolean isClosed=false;

    public ServoMicroSubsystem(RobotHardware robot){
        this.robot=robot;
    }
    public void setMicroServo1(double pos){
        robot.MicroServo1.setPosition(pos);
    }
    public void setMicroServo2(double pos){
        robot.MicroServo2.setPosition(pos);
    }
    //public void setMicroServo12(double pos1, double pos2){
    //    robot.MicroServo1.setPosition(pos1);
    //    robot.MicroServo2.setPosition(pos2);
    //}

    public void setMicroServo12(){
        if(!isClosed) {
            robot.MicroServo1.setPosition(robot.MicroServoINCHIS1);
            robot.MicroServo2.setPosition(robot.MicroServoINCHIS2);
            isClosed=true;
        }
        else {
            robot.MicroServo1.setPosition(robot.MicroServoDESCHIS1);
            robot.MicroServo2.setPosition(robot.MicroServoDESCHIS2);
            isClosed=false;
        }
    }
}
