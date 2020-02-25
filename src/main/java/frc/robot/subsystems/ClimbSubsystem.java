package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ClimbSubsystem extends Subsystem {
    NetworkTableInstance networkTableInst;
    NetworkTableEntry liftPercentage;
    NetworkTableEntry climbPercentage;
    double percentCompleteClimb; //actually climbing with robot
    double percentCompleteLift; //lifting the hook up
    private WPI_TalonSRX climbController = new WPI_TalonSRX(Constants.ID_climbMotor);
  
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

    void OperateClimb(boolean operate) {
        switch (climbState){
            case Idle:
                percentCompleteClimb = 0.0;
                percentCompleteLift = 0.0;
            
                if(operate){
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * Constants.desiredClimberRotationsUp);
                    liftMotorEncoder.reset();
                    climbState = ClimbState.LiftHook; 
                }
                break;
            case LiftHook:
                if(climbController.getClosedLoopError() < Constants.tolerableUnitsFromMaxClimberValue){
                    climbState = ClimbState.HookAtTop ;
                }
                else if(!operate){
                    stopMotor();
                }
                else{
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * Constants.desiredClimberRotationsUp);
                }
                percentCompleteLift = liftMotorEncoder.get()/1000;
                break ;
            case HookAtTop:
                if(operate){
                    climbController.set(ControlMode.Position, Constants.unitsPerRotation * -Constants.desiredClimberRotationsDown);
                    climbState = ClimbState.LiftingBot ;
                }
                break;
            case LiftingBot:
                liftMotorEncoder.reset();
                if(climbController.getClosedLoopError() < Constants.tolerableUnitsFromMaxClimberValue){
                    stopMotor();
                    climbState = ClimbState.Stopped ;
                }
                percentCompleteClimb = liftMotorEncoder.get()/1000;
                break ;
            case Stopped:
                //Bot is immobile
                break ;
        }
            climbPercentage.setNumber(percentCompleteClimb);
            climbPercentage = networkTableInst.getTable("ClimbData").getEntry("climbPercentage");
            liftPercentage.setNumber(percentCompleteLift);
            liftPercentage = networkTableInst.getTable("ClimbData").getEntry("liftPercentage");

    }



    public void moveMotor(double speed){
        climbController.set(speed);
    }

    

    public void stopMotor(){
        climbController.set(0);
    }
}