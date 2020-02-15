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
    public double getForwardSpeed(){return controller.getRawAxis(1);} //Left-side Y axis
    public double getTurnAngle(){return controller.getRawAxis(2);}   //Right-side X axis

    // public boolean getSquareButton(){return controller.getRawButton(1);}
    public boolean getCircleButton(){return controller.getRawButton(3);}
    public boolean getXButton(){return controller.getRawButton(2);}
    public boolean getTriangleButton(){return controller.getRawButton(4);}

    public boolean isAutoTargeting() {return controller.getRawButton(5);} //Top left trigger
    public boolean isBallChasing() {return controller.getRawButton(7);} //Bottom left trigger

    public boolean getIsShooting() {return controller.getRawButton(6);}     // Top right trigger
    public boolean getDeployIntakeManual() {return controller.getRawButton(8);} //Bottom right trigger

    public boolean getPOVUp(){return controller.getPOV() == 1;}
    public boolean getPOVDown(){return controller.getPOV() == 5;}
    public boolean getPOVLeft(){return controller.getPOV() == 3;}
    public boolean getPOVRight(){return controller.getPOV() == 7;}

    //Box buttons and axes
    public boolean getPinkButton(){return box.getRawButton(3);}
    public boolean getTealButton(){return box.getRawButton(4);}
    public boolean getRedButton(){return box.getRawButton(2);}
    public boolean getPurpleButton(){return box.getRawButton(1);}

    public boolean isClimbing(){return box.getRawButton(5);}
    public boolean isOperatingWheel(){return box.getRawButton(6);}

}