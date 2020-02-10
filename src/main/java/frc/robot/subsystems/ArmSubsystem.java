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
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  OI oi = new OI();
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  int colourChangeCounter = 0 ;

  String currentColour= " ";
  String lastColour= " ";

  private Servo armServo = new Servo(0);
  private Spark spinMotor = new Spark(1);

  long waitStartTime ;
 
  enum ArmState { Idle, DeployingArm, SpinningWheel, WaitingForTimeout, RetractingArm } ;
  ArmState armState ;
   
  public ArmSubsystem() {
      armState = ArmState.Idle ;
  }
 
  @Override
  public void initDefaultCommand() {
    armServo.set(0.0) ;
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
 
  
  public void OperateControlPanel() {
    switch (armState) {
      case Idle:
          spinMotor.set(0.0) ;
         armServo.set(0.0) ;
         if (oi.getSquareButton()) {
           armServo.setAngle(Constants.deployAngle);
           armState = ArmState.DeployingArm ;
         }
         break ;
      case DeployingArm:
          if (!oi.getSquareButton()){
            armState = ArmState.Idle;
          } 
          else if (armServo.getAngle() >= 89) {
            armState = ArmState.SpinningWheel;
            colourChangeCounter = 0 ;
            lastColour = getColour() ;
            spinMotor.set(Constants.maxSpinSpeed ) ;
          }
          break ;
      case SpinningWheel:
        if (!oi.getSquareButton()){
          armState = ArmState.Idle;
        } 
         else{
            currentColour = getColour() ;
            if(currentColour!=lastColour){
             colourChangeCounter++;
            }
            if(colourChangeCounter > Constants.minColorChangeCountGoal ){
              waitStartTime = System.currentTimeMillis();
              armState = ArmState.WaitingForTimeout ;
              spinMotor.set(0.0) ;
            } 
          }
         break;
      case WaitingForTimeout:
         if (!oi.getSquareButton()){
            armState = ArmState.Idle;
          } 
         else if ( System.currentTimeMillis() - waitStartTime > Constants.wheelWaitTime ) {
           armServo.set(0.0) ;
           armState = ArmState.RetractingArm ;
         }
         break;
      case RetractingArm:
         if ( armServo.getAngle() < 5){
          armState= ArmState.Idle;
         }
         break;

         
    }
   
  }


  public char getFMSColour() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      return gameData.charAt(0);
    } else {
      return ' ';
    }
    //   switch (gameData.charAt(0))
  //   {
  //     case 'B' :
  //       //Blue case code
  //       break;
  //     case 'G' :
  //       //Green case code
  //       break;
  //     case 'R' :
  //       //Red case code
  //       break;
  //     case 'Y' :
  //       //Yellow case code
  //       break;
  //     default :
  //       //This is corrupt data
  //       break;
  //   }
  // } else {
  //   //Code for no data received yet
  // }

  }

  public String getColour(){
    return "a";
  }

  public int colourChange(){
    return 3;
  }


 
  
 
 
}
 
 

