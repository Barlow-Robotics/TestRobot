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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
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
  private DigitalInput ballIntakeSensor, ballExitSensor;
  private DigitalOutput ballIntakeTransmitter, ballExitTransmitter;
  private boolean prevIntakeValue, prevExitValue;
  enum IndexingState{
    Idle,
    Feeding,
    ProcessingIntake
  }
  
  IndexingState indexingState;
  

  public IndexingSubsystem(){
    indexingWheelDriver = new WPI_TalonSRX(Constants.ID_shooterFeedMotor);
    agitatorMotor = new Spark(Constants.PWMPORT_intakeMotorPort);
    cellCount = 0;
    indexingState = IndexingState.Idle;

    ballExitSensor = new DigitalInput(Constants.DIOPORT_exitSensor);
    ballExitTransmitter = new DigitalOutput(Constants.DIOPORT_exitTransmitter);
    ballIntakeSensor = new DigitalInput(Constants.DIOPORT_intakeSensor);
    ballIntakeTransmitter = new DigitalOutput(Constants.DIOPORT_intakeTransmitter);
  } 



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
  public void operateIndex(boolean firing, boolean cellChasing){
    keepTransmittersActive();
    switch(indexingState){
      case Idle:
        if(firing)
          indexingState = IndexingState.Feeding;
        else if(cellChasing)
          indexingState = IndexingState.ProcessingIntake;
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
          if(ballHasExited())
            cellCount--;
        }
        updatePreviousValues();
        break;
      case ProcessingIntake:
        if(!cellChasing){
          agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          if(ballHasEntered())
            cellCount++;
          agitatorMotor.set(Constants.agitatingSpeed);
        }
      updatePreviousValues();
    } 
  }


  private boolean ballHasExited(){
    return prevExitValue != ballExitSensor.get() && !ballExitSensor.get();
  }


  private boolean ballHasEntered(){
    return prevIntakeValue != ballIntakeSensor.get() && !ballIntakeSensor.get();
  }


  private void updatePreviousValues(){
    prevExitValue = ballExitSensor.get();
    prevIntakeValue = ballIntakeSensor.get();
  } 


  private void keepTransmittersActive(){
    ballIntakeTransmitter.set(true);
    ballExitTransmitter.set(true);
  }


  
  public int getCellCount(){return cellCount;}
}
