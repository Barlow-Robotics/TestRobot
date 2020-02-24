/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX shooterController = new WPI_TalonSRX(Constants.ID_shooterMotor);
  // private WPI_TalonFX falconController = new WPI_TalonFX(Constants.shooterMotorTalonID);
  // private Servo angleServo = new Servo(0);
  private double flywheelSpeedPercent = 0;
  private double servoAngle = 0;
  private NetworkTableInstance networkTableInst;
  private NetworkTableEntry distanceToTarget;
  private NetworkTableEntry kF_Shooter;
  private NetworkTableEntry kP_Shooter;
  private NetworkTableEntry kI_Shooter;
  private NetworkTableEntry kD_Shooter;


  public enum ShooterState{
    IdleSpin,
    Targeting
  }

  private ShooterState shooterState;

  public ShooterSubsystem(NetworkTableInstance networkTableInst){
    shooterState = ShooterState.IdleSpin;

    shooterController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime);
    // falconController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.mainFeedbackLoop, Constants.timeoutTime);
    
    this.networkTableInst = networkTableInst;
    distanceToTarget = networkTableInst.getTable("shooter").getEntry("distanceToTarget");

    kF_Shooter = networkTableInst.getTable("shooter").getEntry("kF_Shooter");
    kP_Shooter = networkTableInst.getTable("shooter").getEntry("kP_Shooter");
    kI_Shooter = networkTableInst.getTable("shooter").getEntry("kI_Shooter");
    kD_Shooter = networkTableInst.getTable("shooter").getEntry("kD_Shooter");

    kF_Shooter.setNumber(Constants.DrivetrainKf);
    kP_Shooter.setNumber(0.02);
    kI_Shooter.setNumber(0);
    kD_Shooter.setNumber(0);

    // shooterController.configMotionAcceleration();
    shooterController.configMotionCruiseVelocity(8192 * 10000);
    shooterController.config_kF(0, (double)kF_Shooter.getNumber(Constants.DrivetrainKf)); 
    shooterController.config_kP(0, (double)kP_Shooter.getNumber(0.02));
    shooterController.config_kI(0, (double)kI_Shooter.getNumber(0.0));
    shooterController.config_kD(0, (double)kD_Shooter.getNumber(0.0));
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void operateShooter(boolean doSpin){
    switch(shooterState){
      case IdleSpin:
        if(doSpin){
          updatePIDValues();
          shooterController.set(ControlMode.Velocity, -Constants.maxShooterSpeed); //CHANGE
          // falconController.set(ControlMode.Velocity, -Constants.maxShooterSpeed);
          SmartDashboard.putNumber("Shooter Flywheel Speed", shooterController.get());
        }
        else {
          shooterController.set(0.0);
          // falconController.set(ControlMode.Velocity, 0);
        }
          
      //   if()
      //     shooterState = ShooterState.Targeting;
      // case Targeting:
      //   if(!oi.isAutoTargeting())
      //     shooterState = ShooterState.IdleSpin;
      //   else{
      //     shooterController.set(ControlMode.Velocity, Constants.maxShooterPercent);
      //     // falconController.set(ControlMode.Velocity, Constants.maxShooterSpeed);
      //     // INSERT ANGLE ADJUSTMENT CODE HERE
      //   }
    }
  }

  

  private void log(){
    SmartDashboard.putNumber("Current Speed", flywheelSpeedPercent);
    SmartDashboard.putNumber("Current Angle", servoAngle * 90);
  }



  private double threshold(double speed){
    if(speed > Constants.maxShooterSpeed)
      speed = Constants.maxShooterSpeed;
    else if(speed < 0)
      speed = 0;
    //else if(speed < Constants.minShooterSpeed)
      //speed = 0;
    //else if(speed > -Constants.minShooterSpeed)
      //speed = 0;

    return speed;
  }



  public ShooterState getShooterState(){
    return shooterState;
  }



  private void updatePIDValues(){
    shooterController.config_kF(0, (double)kF_Shooter.getNumber(Constants.DrivetrainKf)); 
    shooterController.config_kP(0, (double)kP_Shooter.getNumber(0.69));
    shooterController.config_kI(0, (double)kI_Shooter.getNumber(0.0));
    shooterController.config_kD(0, (double)kD_Shooter.getNumber(0.0));
    if(System.currentTimeMillis()%1000 <= 20)
      System.out.println("kP of Shooter:" + kP_Shooter.getNumber(0.02));
  }
}




/*
public void operateShooterWithButtonAdjustments(){
    // Gonna leave trigger commands in here for the moment for testing purposes
    switch(shooterState){
      case IdleSpin:
        checkAndMoveMotor();
        if(oi.getPOVDown() || oi.getPOVUp())
          shooterState = ShooterState.Targeting;
        break;
      case Targeting:
        if(oi.getPOVUp())
          servoAngle += 0.005;
        else if(oi.getPOVDown())
          servoAngle -= 0.005;
        else if(oi.getPOVLeft())
          servoAngle = Constants.setAngleOfServo;
        else
          shooterState = ShooterState.IdleSpin;
        checkAndMoveMotor();
        // angleServo.set(servoAngle);
    }
  }

  private void checkAndMoveMotor(){
    if(oi.getIsShooting())
      flywheelSpeedPercent += 0.05;
    else
      flywheelSpeedPercent -= 0.05;
        
    if(oi.getXButton())
      flywheelSpeedPercent = 0;

      flywheelSpeedPercent = threshold(flywheelSpeedPercent);
    shooterController.set(flywheelSpeedPercent);
    log();
  }

*/