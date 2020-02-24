/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */


public class IntakeSubsystem extends Subsystem {
  // private Solenoid intakeDeploy, intakeRetract;
  private Spark intakeMotor;
  private boolean priorOperatorInput = false;

  public enum IntakeState{
    RetractedIdle,
    Deploying,
    Intaking,
    Retracting
  }
  
  IntakeState intakeState;

  public IntakeSubsystem(){
    // intakeDeploy = new Solenoid(Constants.intakeDeployPort);
    // intakeRetract = new Solenoid(Constants.intakeRetractPort);
    intakeMotor = new Spark(Constants.PWMPORT_intakeMotorPort);
    intakeState = IntakeState.RetractedIdle;
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



  public void operateIntake(boolean intake){

    switch(intakeState){
      case RetractedIdle:
        if(intake){
          // setIntakeDeploy(true);
          intakeState = IntakeState.Deploying;
        }
        break;
      case Deploying:
          intakeMotor.set(Constants.intakeSpeed);
          intakeState = IntakeState.Intaking;
        break;
      case Intaking:
        if(!intake){
          intakeState = IntakeState.Retracting;
        }
        else if(intake){
          intakeMotor.set(-0.75);
        }
        break;
      case Retracting:
        intakeMotor.set(0);
        // setIntakeDeploy(false);
        intakeState = IntakeState.RetractedIdle;
        break;
    }
  }



  public IntakeState getIntakeState(){
    return intakeState;
  }


  // private void setIntakeDeploy(boolean deploy){
  //   intakeDeploy.set(deploy);
  //   intakeRetract.set(!deploy);

  // }
}
