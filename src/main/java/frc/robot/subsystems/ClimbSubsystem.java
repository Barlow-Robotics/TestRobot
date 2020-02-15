package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.OI;

public class ClimbSubsystem extends Subsystem {
    OI oi = new OI();
    private WPI_TalonSRX climbController = new WPI_TalonSRX(Constants.ClimbMotorPortNumber);
  
    Encoder liftMotorEncoder ;

    public ClimbSubsystem(){
      // liftMotorEncoder = new Encoder(Constants.LiftEncoderChannelA, Constants.LiftEncoderChannelB);

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
                if(oi.getSquareButton()){
                    moveMotor(Constants.LiftMotorSpeed);
                    liftMotorEncoder.reset();
                    climbState = ClimbState.LiftHook ; 
                }
                break ;
            case LiftHook:
                if(liftMotorEncoder.get() >= Constants.MaxEncoderValue){
                    stopMotor();
                    climbState = ClimbState.HookAtTop ;
                }
                break ;
            case HookAtTop:
                if(oi.getXButton()){
                    moveMotor(-Constants.LiftMotorSpeed);
                    liftMotorEncoder.reset();
                    climbState = ClimbState.LiftingBot ;
                }
                break ;
            case LiftingBot:
                if(liftMotorEncoder.get() <= 0){
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