package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.OI;

public class ClimbSubsystem extends Subsystem {
    OI oi = new OI();
    private WPI_TalonSRX climbController = new WPI_TalonSRX(6);
    private double speed = 0.32;

    public ClimbSubsystem(){}

    @Override
	protected void initDefaultCommand() {
    }
    
    public void moveUp(){
       if (oi.getRTopTrigger() == true){
        climbController.set(speed);
       }
    }
    public void moveDown(){
        if (oi.getRBottomTrigger() == true){
            speed = -speed;
            climbController.set(speed);
           }
    }
    
    public void stopClimb(){
        speed = 0;
      }



}