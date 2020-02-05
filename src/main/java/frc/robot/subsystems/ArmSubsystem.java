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
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark rotationControl = new Spark(Constants.wheelRotationSparkPort);
  private Spark deploymentControl = new Spark(Constants.wheelDeploymentSparkPort);

  int minValue= 3;


  enum ArmState { Idle, DeployingArm, SpinningWheel, WaitingForTimeout, RetractingArm } ;
  ArmState armState ;
   
  public ArmSubsystem() {
      armState = ArmState.Idle ;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



  public void OperateControlPanel() {
    switch (armState) {
      case Idle:
         if ( getDeployButton() ) {
           // start the small block motor
           armState = ArmState.DeployingArm ;
         }
         break ;
      case DeployingArm:
         if(limitSwitchDeploy()){
           //stop the small block motor
           armState = ArmState.SpinningWheel ;
         }
         break ;
      case SpinningWheel:
         //turn on wheel motor go spinnoe
         //change colour change count to 0
         if(colourChange()>minValue){
            //stop wheel
            armState = ArmState.WaitingForTimeout ;
         }
         break;
      case WaitingForTimeout:
         
         
        //
    }
    
  }

  
  private boolean limitSwitchDeploy() {
    return false;
  }

  private int colourChange(){
    return 3;
  }


  private boolean getDeployButton() {
      return false;
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

