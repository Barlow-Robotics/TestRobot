/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Spark;



/**
 * Add your docs here.
 */
public class IndexingSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //private WPI_TalonSRX wheelDriver;
  private WPI_TalonSRX indexingWheelDriver;
  private Spark agitatorMotor;
  private int cellCount;
  enum IndexingState{
    Idle,
    Feeding,
    Agitating
  }
  
  IndexingState indexingState;
  

  public IndexingSubsystem(){
    indexingWheelDriver = new WPI_TalonSRX(Constants.ID_shooterFeedMotor);
    agitatorMotor = new Spark(Constants.PWMPORT_intakeMotorPort);
    cellCount = 0;
    indexingState = IndexingState.Idle;
  } 



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
  public void operateIndex(boolean firing, boolean cellChasing){
    switch(indexingState){
      case Idle:
        if(firing)
          indexingState = IndexingState.Feeding;
        else if(cellChasing)
          indexingState = IndexingState.Agitating;
        else {
          indexingWheelDriver.set(0.0);
          agitatorMotor.set(0.0);
        }
        break;
      case Feeding:
        if(!firing){
          indexingWheelDriver.set(0);
          agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          indexingWheelDriver.set(-Constants.feedingSpeed);
          // agitatorMotor.set(Constants.agitatingSpeed);
          if(false/*Sensor detects ball exit*/)
            cellCount--;
          if(false/*Other sensors detect ball entry*/)
            cellCount++;
        }
        break;
      case Agitating:
        if(!cellChasing){
          agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          agitatorMotor.set(Constants.agitatingSpeed);
        }
    } 
  }

  public int getCellCount(){return cellCount;}
}
