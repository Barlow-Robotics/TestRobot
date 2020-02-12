package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI{
    public static boolean getXButton;
	public static boolean getSquareButton;
    private Joystick controller;
    private Joystick box;
    
    public OI(){
        controller = new Joystick(Constants.joystickPort);
        box = new Joystick(Constants.boxControlPort);
    }

    //Joystick buttons and axes
    public double getLeftY(){return controller.getRawAxis(1);}
    public double getRightX(){return controller.getRawAxis(2);}

    public boolean getSquareButton(){return controller.getRawButton(1);}
    public boolean getCircleButton(){return controller.getRawButton(3);}
    public boolean getXButton(){return controller.getRawButton(2);}

    public boolean getLTopTrigger() {return controller.getRawButton(5);}
    public boolean getLBottomTrigger() {return controller.getRawButton(7);}

    public boolean getRTopTrigger() {return controller.getRawButton(6);}
    public boolean getRBottomTrigger() {return controller.getRawButton(8);}

    public boolean getPOVUp(){return controller.getPOV() == 1;}
    public boolean getPOVDown(){return controller.getPOV() == 5;}
    public boolean getPOVLeft(){return controller.getPOV() == 3;}
    public boolean getPOVRight(){return controller.getPOV() == 7;}

    //Box buttons and axes
    public boolean getPinkButton(){return box.getRawButton(3);}
    public boolean getTealButton(){return box.getRawButton(4);}
    public boolean getRedButton(){return box.getRawButton(2);}
    public boolean getPurpleButton(){return box.getRawButton(1);}

    

    public void publishData(double data){
        SmartDashboard.putNumber("Deploy Speed", data);
    }

}