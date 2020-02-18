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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.OI;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX shooterController = new WPI_TalonSRX(Constants.shooterMotorTalonID);
  // private WPI_TalonFX falconController = new WPI_TalonFX(Constants.shooterMotorTalonID);
  private Servo angleServo = new Servo(0);
  private double flywheelSpeedPercent = 0;
  private double servoAngle = 0;
  private OI oi;

  public enum ShooterState{
    IdleSpin,
    Targeting
  }

  private ShooterState shooterState;

  public ShooterSubsystem(){
    shooterState = ShooterState.IdleSpin;

    shooterController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime);
    // falconController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.mainFeedbackLoop, Constants.timeoutTime);
    
    oi = new OI();
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
          shooterController.set(-Constants.maxShooterPercent); //CHANGE
          //falconController.set(ControlMode.Velocity, -Constants.maxShooterPercent);
          SmartDashboard.putNumber("Shooter Flywheel Speed", shooterController.get());
        }
        else 
          shooterController.set(0.0);
          
        if(oi.isAutoTargeting())
          shooterState = ShooterState.Targeting;
      case Targeting:
        if(!oi.isAutoTargeting())
          shooterState = ShooterState.IdleSpin;
        else{
          shooterController.set(ControlMode.Velocity, Constants.maxShooterSpeed);
          // INSERT ANGLE ADJUSTMENT CODE HERE
        }
    }
  }

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
        angleServo.set(servoAngle);
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
}
