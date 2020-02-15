/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.OI;

/**
 * Add your docs here.
 */



public class IntakeSubsystem extends Subsystem {
  private Solenoid intakeDeploy, intakeRetract;
  private OI oi;

  public enum IntakeState{
    RetractedIdle,
    Deploying,
    Intaking,
    Retracting
  }
  
  IntakeState intakeState;

  public IntakeSubsystem(){
    oi = new OI();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void operateIntake(){

    switch(intakeState){

    }
  }

  public IntakeState getIntakeState(){
    return intakeState;
  }
}
