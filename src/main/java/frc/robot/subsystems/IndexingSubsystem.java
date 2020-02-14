/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Spark;

import frc.robot.Constants;
import frc.robot.OI;



/**
 * Add your docs here.
 */
public class IndexingSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //private WPI_TalonSRX wheelDriver;
  private Spark indexingWheelDriver;
  private Spark agitatorMotor;
  private OI oi;
  private int cellCount;
  enum IndexingState{
    Idle,
    Feeding
  }
  
  IndexingState indexingState;
  

  public IndexingSubsystem(){
//    wheelDriver = new WPI_TalonSRX(Constants.indexingWheelMotor);
    indexingWheelDriver = new Spark(Constants.indexingWheelMotor);
    oi = new OI();
    cellCount = 0;
    indexingState = IndexingState.Idle;
  } 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void operateIndex(){
    
    switch(indexingState){
      case Idle:
        if(isShooting())
          indexingState = IndexingState.Feeding;
        else
          indexingWheelDriver.set(0.0);
        break;
      case Feeding:
        if(!isShooting()){
          indexingWheelDriver.set(0);
          // agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          indexingWheelDriver.set(Constants.feedingSpeed);
          // agitatorMotor.set(Constants.agitatingSpeed);
          if(false/*Sensor detects ball exit*/)
            cellCount--;
        }
    }
  }


  private boolean isShooting(){
    return oi.getIsShooting();
  }
}
