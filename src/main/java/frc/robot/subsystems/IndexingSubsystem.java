/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private WPI_TalonSRX shooterController;
  private Spark agitatorMotor;
  private int cellCount;
  private BeamSensor exitSensor;
  private BeamSensor entrySensor;
  private boolean prevIntakeValue, prevExitValue;
  private boolean agitating;
  private double agitatingCycleStartTime;
  private double cycleStart;
  private double firingStartTime;


  private double flywheelSpeedPercent = 0;
  private double servoAngle = 0;
  private NetworkTableEntry distanceToTarget;
  private NetworkTableEntry kF_Shooter;
  private NetworkTableEntry kP_Shooter;
  private NetworkTableEntry kI_Shooter;
  private NetworkTableEntry kD_Shooter;



  enum IndexingState{
    Idle,
    SpinningUp,
    Firing,
    Agitating,
    ManualIntake
  }

  enum AgitatingState{
    Clockwise, 
    Counterclockwise, 
    Off
  }
  
  IndexingState indexingState;
  AgitatingState agitatingState;

  NetworkTableInstance networkTableInst;
  private NetworkTableEntry kF_Index;
  private NetworkTableEntry kP_Index;
  private NetworkTableEntry kI_Index;
  private NetworkTableEntry kD_Index;
  private NetworkTableEntry cellCountEntry;
  private NetworkTableEntry targetDistanceEntry ;



  public IndexingSubsystem(NetworkTableInstance networkTableInst, int cellCount){
    indexingWheelDriver = new WPI_TalonSRX(Constants.ID_shooterFeedMotor);
    shooterController = new WPI_TalonSRX(Constants.ID_shooterMotor);
    agitatorMotor = new Spark(Constants.PWMPORT_agitatorMotor);
    this.cellCount = cellCount;
    indexingState = IndexingState.Idle;

    this.networkTableInst = networkTableInst;
    kF_Index = networkTableInst.getTable("shooter").getEntry("kF_Index");
    kP_Index = networkTableInst.getTable("shooter").getEntry("kP_Index");
    kI_Index = networkTableInst.getTable("shooter").getEntry("kI_Index");
    kD_Index = networkTableInst.getTable("shooter").getEntry("kD_Index");
    cellCountEntry = networkTableInst.getTable("shooter").getEntry("cellCount");

    kF_Index.setNumber(Constants.indexingkF);
    kP_Index.setNumber(Constants.indexingkP);
    kI_Index.setNumber(0.000);
    kD_Index.setNumber(Constants.indexingkD);
    cellCountEntry.setNumber(cellCount);

    indexingWheelDriver.setNeutralMode(NeutralMode.Coast);
    indexingWheelDriver.configMotionCruiseVelocity(8192 * 10000);
    indexingWheelDriver.config_kF(0, (double)kF_Index.getNumber(Constants.indexingkF)); 
    indexingWheelDriver.config_kP(0, (double)kP_Index.getNumber(Constants.indexingkP));
    indexingWheelDriver.config_kI(0, (double)kI_Index.getNumber(0.000));
    indexingWheelDriver.config_kD(0, (double)kD_Index.getNumber(Constants.indexingkD));

    shooterController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime);
    shooterController.configNominalOutputForward(0.0);
    shooterController.configNominalOutputReverse(0.0);
    shooterController.configPeakOutputForward(1.0);
    shooterController.configPeakOutputReverse(-1.0);

    this.networkTableInst = networkTableInst;
    distanceToTarget = networkTableInst.getTable("shooter").getEntry("distanceToTarget");

    kF_Shooter = networkTableInst.getTable("shooter").getEntry("kF_Shooter");
    kP_Shooter = networkTableInst.getTable("shooter").getEntry("kP_Shooter");
    kI_Shooter = networkTableInst.getTable("shooter").getEntry("kI_Shooter");
    kD_Shooter = networkTableInst.getTable("shooter").getEntry("kD_Shooter");

    kF_Shooter.setNumber(Constants.shooterkF);
    kP_Shooter.setNumber(Constants.shooterkP);
    kI_Shooter.setNumber(0.000);
    kD_Shooter.setNumber(Constants.shooterkD);

    targetDistanceEntry = networkTableInst.getTable("vision").getEntry("targetDistance") ;

    exitSensor = new BeamSensor(Constants.DIOPORT_exitSensor, Constants.DIOPORT_exitTransmitter);
    entrySensor = new BeamSensor(Constants.DIOPORT_intakeSensor, Constants.DIOPORT_intakeTransmitter);
    
    shooterController.configMotionCruiseVelocity(8192 * 10000);
    shooterController.config_kF(0, (double)kF_Shooter.getNumber(Constants.shooterkF)); 
    shooterController.config_kP(0, (double)kP_Shooter.getNumber(Constants.shooterkP));
    shooterController.config_kI(0, (double)kI_Shooter.getNumber(0.000));
    shooterController.config_kD(0, (double)kD_Shooter.getNumber(Constants.shooterkD));

    agitating = true;
    agitatingCycleStartTime = 0;
    cycleStart = 0;

    firingStartTime = 0.0;
  } 



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
  //OO COMMENTED OUT ALL AGITATOR MOTOR REFERENCES EXCEPT FOR TOP
  public void operateIndex(boolean firing, boolean manualIntake, boolean cellChasing, boolean agitate){
    double agitatorSpeed = Constants.agitatingSpeed;
    if(!agitate)
      agitatorSpeed = 0.0;
    else
      agitatorSpeed = Constants.agitatingSpeed;
    agitatorMotor.set(agitatorSpeed);
    // wpk need to clean up all these magic numbers
    double shooterSpeedScale = Constants.desiredShooterPercent ;
    double distanceToTarget = targetDistanceEntry.getDouble( -1.0) ;
    if ( distanceToTarget > 0.0) {
      //OO FIX SHOOTER PERCENT HERE
      shooterSpeedScale = Constants.desiredShooterPercent +  0.10 * ((( distanceToTarget / 1000) * 3.281) -10.0 ) / ( 31.0 -10.0) ;
    }
    keepTransmittersActive();
    updatePIDValues();
    switch(indexingState){
      case Idle:
        if(firing){
          indexingState = IndexingState.SpinningUp;
          agitatingState = AgitatingState.Clockwise;
          agitatingCycleStartTime = System.currentTimeMillis();
//        shooterController.set(ControlMode.Velocity, Constants.desiredShooterPercent * Constants.maxShooterSpeed);
          shooterController.set(ControlMode.Velocity, shooterSpeedScale * Constants.maxShooterSpeed);
          firingStartTime = System.currentTimeMillis();
        }
        else if(cellChasing)
          indexingState = IndexingState.Agitating;
        else if(manualIntake)
          indexingState = IndexingState.ManualIntake;
        else {
          indexingWheelDriver.set(0.0);
          // agitatorMotor.set(0.0);
          shooterController.set(0.0);
        }
        break;
      case SpinningUp:
        if(shooterController.getSelectedSensorVelocity() >= Constants.minShooterPercentForFiring * shooterController.getClosedLoopTarget()
           || System.currentTimeMillis() - firingStartTime >= Constants.shooterSpinupTimeout){
          indexingState = IndexingState.Firing;
        }
        else if(!firing){
          shooterController.set(0.0);
          indexingState = IndexingState.Idle;
        }
        // shooterController.set(ControlMode.Velocity, Constants.desiredShooterPercent * Constants.maxShooterSpeed);
        shooterController.set(ControlMode.Velocity, shooterSpeedScale * Constants.maxShooterSpeed);
        break;
      case Firing:
        if(!firing){
          indexingWheelDriver.set(0);
          shooterController.set(0.0);
          // agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
          agitatingState = AgitatingState.Off;
        }
        else{
          indexingWheelDriver.set(ControlMode.Velocity, Constants.desiredFeedingPercent * Constants.maxFeedingVelocity);
//          shooterController.set(ControlMode.Velocity, Constants.desiredShooterPercent * Constants.maxShooterSpeed);
          shooterController.set(ControlMode.Velocity, shooterSpeedScale * Constants.maxShooterSpeed);
          // if(agitating)
          //   agitatorMotor.set(agitatorSpeed);
          // else
            // agitatorMotor.set(0.0);

          if(ballHasExited())
            cellCount--;
        }
        updatePreviousValues();
        break;
      case Agitating:
        if(!cellChasing){
          // agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          if(ballHasEntered())
            cellCount++;
          // agitatorMotor.set(agitatorSpeed);
        }
      break;
      case ManualIntake:
        if(!manualIntake){
          // agitatorMotor.set(0);
          indexingState = IndexingState.Idle;
        }
        else{
          if(ballHasEntered())
            cellCount++;
          // agitatorMotor.set(agitatorSpeed);
        }
      break;
    } 
    SmartDashboard.putNumber("Ball Count", cellCount);
    cellCountEntry.setNumber(cellCount);
    updatePreviousValues();
  }


  public void manualFeed(boolean spin, double speed){
    if(spin){
      indexingWheelDriver.set(speed);
      SmartDashboard.putNumber("Measured indexing speed", indexingWheelDriver.getSelectedSensorVelocity());
    }
    else
      indexingWheelDriver.set(0.0);
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
    indexingWheelDriver.config_kF(0, (double)kF_Index.getNumber(Constants.indexingkF)); 
    indexingWheelDriver.config_kP(0, (double)kP_Index.getNumber(Constants.indexingkP));
    indexingWheelDriver.config_kI(0, (double)kI_Index.getNumber(0.000));
    indexingWheelDriver.config_kD(0, (double)kD_Index.getNumber(Constants.indexingkD));
  }


  
  public int getCellCount(){return cellCount;}
}
