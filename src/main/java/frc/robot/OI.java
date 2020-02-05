package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI{
    private Joystick controller;
    public OI(){
        controller = new Joystick(Constants.joystickPort);
    }

    public double getLeftStick(){
        return controller.getRawAxis(1);
    }

    public double getRightStick(){
        return controller.getRawAxis(5);
    }

    public boolean getSquareButton(){
        return controller.getRawButton(1);
    }

    public boolean getCircleButton(){
        return controller.getRawButton(3);
    }

    public boolean getXButton(){
        return controller.getRawButton(2);
    }

    public boolean getLTopTrigger() {
        return controller.getRawButton(5);
    }

    public boolean getLBottomTrigger() {
        return controller.getRawButton(7);
    }

    public boolean getRTopTrigger() {
        return controller.getRawButton(6);
        //climb - up
    }

    public boolean getRBottomTrigger() {
        return controller.getRawButton(8);
        //climb - move down
    }

    public void publishData(double data){
        SmartDashboard.putNumber("Deploy Speed", data);
    }

}