/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX shooterController = new WPI_TalonSRX(Constants.shooterMotor);
  private double speed = 0;

  public ShooterSubsystem(){}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveMotor(){
    speed = threshold(speed);
    shooterController.set(speed);
    logSpeed();
  }

  public void stopMotor(){
    speed = 0;
  }

  public void increaseSpeed(boolean tenths){
    if(tenths)
      speed += 0.05;
    else
      speed += 0.01;
  }

  public void decreaseSpeed(boolean tenths){
    if(tenths)
      speed -= 0.05;
    else
      speed -= 0.01;
  }

  private void logSpeed(){
    SmartDashboard.putNumber("Current Speed", speed);
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
}
