/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.components.BeamSensor;



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
  private BeamSensor exitSensor;
  private BeamSensor entrySensor;
  private boolean prevIntakeValue, prevExitValue;
  enum IndexingState{
    Idle,
    Feeding,
    ProcessingIntake
  }
  
  IndexingState indexingState;

  NetworkTableInstance networkTableInst;
  private NetworkTableEntry kF_Index;
  private NetworkTableEntry kP_Index;
  private NetworkTableEntry kI_Index;
  private NetworkTableEntry kD_Index;
  private NetworkTableEntry cellCountEntry;

  public IndexingSubsystem(NetworkTableInstance networkTableInst){
    indexingWheelDriver = new WPI_TalonSRX(Constants.ID_shooterFeedMotor);
    agitatorMotor = new Spark(Constants.PWMPORT_agitatorMotor);
    cellCount = 0;
    indexingState = IndexingState.Idle;

    this.networkTableInst = networkTableInst;
    kF_Index = networkTableInst.getTable("shooter").getEntry("kF_Index");
    kP_Index = networkTableInst.getTable("shooter").getEntry("kP_Index");
    kI_Index = networkTableInst.getTable("shooter").getEntry("kI_Index");
    kD_Index = networkTableInst.getTable("shooter").getEntry("kD_Index");
    cellCountEntry = networkTableInst.getTable("shooter").getEntry("cellCount");

    kF_Index.setNumber(Constants.DrivetrainKf);
    kP_Index.setNumber(0.02);
    kI_Index.setNumber(0);
    kD_Index.setNumber(0);
    cellCountEntry.setNumber(0);

    indexingWheelDriver.configMotionCruiseVelocity(8192 * 10000);
    indexingWheelDriver.config_kF(0, (double)kF_Index.getNumber(Constants.DrivetrainKf)); 
    indexingWheelDriver.config_kP(0, (double)kP_Index.getNumber(0.02));
    indexingWheelDriver.config_kI(0, (double)kI_Index.getNumber(0.0));
    indexingWheelDriver.config_kD(0, (double)kD_Index.getNumber(0.0));

    exitSensor = new BeamSensor(Constants.DIOPORT_exitSensor, Constants.DIOPORT_exitTransmitter);
    entrySensor = new BeamSensor(Constants.DIOPORT_intakeSensor, Constants.DIOPORT_intakeTransmitter);
  } 



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
  public void operateIndex(boolean firing, boolean cellChasing){
    keepTransmittersActive();
    updatePIDValues();
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
          agitatorMotor.set(Constants.agitatingSpeed);
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
      break;
    } 
    if(ballHasEntered()) cellCount++;
    if(ballHasExited()) cellCount--;
    SmartDashboard.putNumber("Ball Count", cellCount);
    cellCountEntry.setNumber(cellCount);
    updatePreviousValues();
  }


  private boolean ballHasExited(){
    return prevExitValue != exitSensor.getSensorValue() && !exitSensor.getSensorValue();
  }


  private boolean ballHasEntered(){
    return prevIntakeValue != entrySensor.getSensorValue() && !entrySensor.getSensorValue();
  }


  private void updatePreviousValues(){
    prevExitValue = exitSensor.getSensorValue();
    prevIntakeValue = entrySensor.getSensorValue();
  } 


  public void postSensorValues(){
    exitSensor.getSensorValue();
    entrySensor.getSensorValue();
  }


  public void setSensorValues(boolean set){
    exitSensor.setOutputValue(set);
    entrySensor.setOutputValue(set);
  }


  private void keepTransmittersActive(){
    entrySensor.setOutputValue(true);
    exitSensor.setOutputValue(true);
  }



  private void updatePIDValues(){
    indexingWheelDriver.config_kF(0, (double)kF_Index.getNumber(Constants.DrivetrainKf)); 
    indexingWheelDriver.config_kP(0, (double)kP_Index.getNumber(0.02));
    indexingWheelDriver.config_kI(0, (double)kI_Index.getNumber(0.0));
    indexingWheelDriver.config_kD(0, (double)kD_Index.getNumber(0.0));
  }


  
  public int getCellCount(){return cellCount;}
}
