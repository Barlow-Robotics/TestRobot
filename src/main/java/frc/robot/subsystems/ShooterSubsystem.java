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
import edu.wpi.first.wpilibj.Encoder;
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
  private Servo angleServo = new Servo(0);
  private double speed = 0;
  private double angle = 0;
  private OI oi;

  public enum ShooterState{
    IdleSpin,
    Targeting
  }

  private ShooterState shooterState;

  public ShooterSubsystem(){
    shooterState = ShooterState.IdleSpin;

    shooterController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    
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

  public void operateShooterOP(){
    // Gonna leave trigger commands in here for the moment for testing purposes
    switch(shooterState){
      case IdleSpin:
        checkAndMoveMotor();
        if(oi.getPOVDown() || oi.getPOVUp())
          shooterState = ShooterState.Targeting;
        break;
      case Targeting:
        if(oi.getPOVUp())
          angle += 0.005;
        else if(oi.getPOVDown())
          angle -= 0.005;
        else if(oi.getPOVLeft())
          angle = Constants.setAngleOfServo;
        else
          shooterState = ShooterState.IdleSpin;
        checkAndMoveMotor();
        angleServo.set(angle);
    }
  }

  private void checkAndMoveMotor(){
    if(oi.getIsShooting())
      speed += 0.05;
    else
      speed -= 0.05;
        
    if(oi.getXButton())
      speed = 0;

    speed = threshold(speed);
    shooterController.set(speed);
    log();
  }

  private void log(){
    SmartDashboard.putNumber("Current Speed", speed);
    SmartDashboard.putNumber("Current Angle", angle * 90);
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
