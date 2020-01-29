/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark rotationControl = new Spark(Constants.wheelRotationSparkPort);
  private Spark deploymentControl = new Spark(Constants.wheelDeploymentSparkPort);

  //DigitalInput limitSwitchDeploy = new DigitalInput(0);
  //DigitalInput limitSwitchRetract = new DigitalInput(1);
  
  //private Encoder deploymentEncoder = new Encoder(1, 2, true, EncodingType.k4X);
   
  public ArmSubsystem(){}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void deploy(){
    //if(/*encoder.distance*/false)
      deploymentControl.set(Constants.maxDeploySpeed);
    //else if (/*encoder.distance*/true)
      //deploymentControl.set(0.0);
    SmartDashboard.putNumber("Speed", deploymentControl.getSpeed());
      
  }

  public void retract(){
    //if(/*encoder.distance < setdistance*/false)
      deploymentControl.set(-Constants.maxDeploySpeed);
    //else if (/*encoder.distance*/true)
      //deploymentControl.set(0.0);
  }

  public void spin(boolean shouldSpin) {
    if (shouldSpin)
      rotationControl.set(Constants.maxSpinSpeed);
    else if (!shouldSpin)
      rotationControl.set(0.0);
  }

  public void stopDeploy(){
    deploymentControl.set(0);
  }

  public double getDeploySpeed() {return deploymentControl.getSpeed();}


}

