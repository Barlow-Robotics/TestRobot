/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */


public class IntakeSubsystem extends Subsystem {
  // private Solenoid intakeDeploy, intakeRetract;
  private Spark intakeMotor;

  public enum IntakeState{
    RetractedIdle,
    Deploying,
    Intaking,
    Retracting,
    DeployedNotSpinning
  }
  
  IntakeState intakeState;
  Solenoid extendIntake, retractIntake;
  boolean prevDeploy;

  public IntakeSubsystem(){
    intakeMotor = new Spark(Constants.PWMPORT_intakeMotorPort);
    intakeState = IntakeState.RetractedIdle;
    extendIntake = new Solenoid(Constants.SOLENOID_extendIntake);
    retractIntake = new Solenoid(Constants.SOLENOID_retractIntake);
    prevDeploy = false;
    extendIntake.set(false);
    retractIntake.set(true);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



  public void operateIntake(boolean intake, boolean deploy, boolean reverse){

    double direction = 1.0 ;
    if ( reverse) {
      direction = -direction ;
    }
    switch(intakeState){
      case RetractedIdle:
        if(intake){
          intakeState = IntakeState.Deploying;
        }
        else if(deploy && prevDeploy != deploy){
          intakeState = IntakeState.DeployedNotSpinning;
          prevDeploy = deploy;
        }
        break;
      case Deploying:
          intakeMotor.set(-Constants.intakeSpeed * direction);
          setIntakeDeploy(true);
          intakeState = IntakeState.Intaking;
        break;
      case Intaking:
        if(!intake){
          intakeState = IntakeState.Retracting;
        }
        else if(intake){
          setIntakeDeploy(true);
          intakeMotor.set(-Constants.intakeSpeed * direction);
        }
        break;
      case Retracting:
        intakeMotor.set(0);
        setIntakeDeploy(false);
        intakeState = IntakeState.RetractedIdle;
        break;
      case DeployedNotSpinning:
        if(deploy && prevDeploy != deploy){
          setIntakeDeploy(false);
          intakeState = IntakeState.RetractedIdle;
        }
        else
          setIntakeDeploy(true);
      break;
    }
    prevDeploy = deploy;
  }



  public void setIntakeDeploy(boolean deploy){
    // wpk FIX ME! Hard code to keep it deployed
    if ( !deploy) {
    extendIntake.set(deploy);
    retractIntake.set(!deploy);
    }
  }



  public IntakeState getIntakeState(){
    return intakeState;
  }
}
