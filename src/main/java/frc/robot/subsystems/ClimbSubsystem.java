package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.OI;

public class ClimbSubsystem extends Subsystem {
    OI oi = new OI();
    private WPI_TalonSRX climbController = new WPI_TalonSRX(Constants.ClimbMotorPortNumber);
  
    Encoder liftMotorEncoder ;

    public ClimbSubsystem(){
    }

    @Override
	protected void initDefaultCommand() {
    }
    
    enum ClimbState { 
        Idle,
        LiftHook,
        HookAtTop,
        LiftingBot,
        Stopped	
    } 

    ClimbState climbState ;

    void OperateClimb() {
        switch (climbState){
            case Idle:
                if(oi.isClimbing()){
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * Constants.desiredClimberRotationsUp);
                    liftMotorEncoder.reset();
                    climbState = ClimbState.LiftHook; 
                }
                break;
            case LiftHook:
                if(climbController.getClosedLoopError() < Constants.tolerableUnitsFromMaxClimberValue){
                    climbState = ClimbState.HookAtTop ;
                }
                else if(!oi.isClimbing()){
                    stopMotor();
                }
                else{
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * Constants.desiredClimberRotationsUp);
                }
                break ;
            case HookAtTop:
                if(oi.isClimbing()){
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * -Constants.desiredClimberRotationsDown);
                    climbState = ClimbState.LiftingBot ;
                }
                break ;
            case LiftingBot:
                if(climbController.getClosedLoopError() < Constants.tolerableUnitsFromMaxClimberValue){
                    stopMotor();
                    climbState = ClimbState.Stopped ;
                }
                break ;
            case Stopped:
                //Bot is immobile
                break ;
        }
    }



    public void moveMotor(double speed){
        climbController.set(speed);
    }

    

    public void stopMotor(){
        climbController.set(0);
    }
}