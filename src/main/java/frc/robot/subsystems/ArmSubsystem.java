/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
//servo port 0 
 
//sure we'll say the motor is port 1
 
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.components.ColourFilter;
 
/**
 * Add your docs here.
 */
 
public class ArmSubsystem extends Subsystem {
  char colourGoal;
 
  OI oi = new OI();
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
  int colourChangeCounter = 0;
 
  char currentColour = 'a';
  char lastColour = 'b';
 
  private final static I2C.Port i2cPort = I2C.Port.kOnboard;
  final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
 
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
 
 
  //put calibrated colours here
 
  private Solenoid armPiston = new Solenoid(0);
  private Spark spinMotor = new Spark(1);
 
  ColourFilter colourFilter;
 
  long waitStartTime;
 
  enum ArmState {
    Idle, DeployingArm, SpinningWheel, WaitingForTimeout, RetractingArm
  };
 
  ArmState armState;
 
  enum SpinningMode{
    Which, TurnTimes, TurnColour
  };
 
  SpinningMode spinningMode;
 
 
  public ArmSubsystem() {
    armState = ArmState.Idle;
    colourFilter = new ColourFilter(Constants.colourFilterLength, 'n');
  }
 
  @Override
  public void initDefaultCommand() {
    spinMotor.set(0.0);
    armPiston.set(false);
  }
 
  public void OperateControlPanel() {
    System.out.println("entered arm");
    switch (armState) {
    case Idle:
      spinMotor.set(0.0);
      armPiston.set(false);
      if (oi.getSquareButton()) { //what button is used ?? MUST CHANGE
        armPiston.set(true);
        armState = ArmState.DeployingArm;
      }
      break;
    case DeployingArm:
      if (!oi.getSquareButton()) {
        armState = ArmState.Idle;
      } else  {
        armState = ArmState.SpinningWheel;
        colourChangeCounter = 0;
        lastColour = colourFilter.getColour();
        spinMotor.set(Constants.maxSpinSpeed);
      }
      break;
    case SpinningWheel:
      if (!oi.getSquareButton()) {
        armState = ArmState.Idle;
      } else {
        switch (spinningMode){
          case Which:
            if (getFMSColour == 'n'){
              spinningMode = SpinningMode.TurnTimes;
            } 
            else{
              spinningMode = SpinningMode.TurnColour;
            }
            break;
          case TurnTimes:
            colourFilter.addMeasurement( getColourFromSensor() );
            currentColour = colourFilter.getColour();
            if (currentColour != lastColour) {
              colourChangeCounter++;
            }
            if (colourChangeCounter > Constants.minColorChangeCountGoal) {
              waitStartTime = System.currentTimeMillis();
              spinMotor.set(0.0);
              armState = ArmState.WaitingForTimeout;
            }
            break;
          case TurnColour:
            if (getColourFromSensor == getFMSColour){
                colourChangeCounter = 0;
                if (colourChangeCounter != 2){
                  if (currentColour != lastColour) {
                    colourChangeCounter++;
                  }  
                }
                else{
                  spinMotor.set(0.0);
                  armState = ArmState.WaitingForTimeout;
                }              
            }
            else{
              while (getColourFromSensor!=getFMSColour){
                spinMotor.set(Constants.maxSpinSpeed);
              }
              colourChangeCounter = 0;
                if (colourChangeCounter != 2){
                  if (currentColour != lastColour) {
                    colourChangeCounter++;
                  }  
                }
                else{
                  spinMotor.set(0.0);
                  armState = ArmState.WaitingForTimeout;
                }              
            }
            
            break;
        }
      }
      break;
    case WaitingForTimeout:
      if (!oi.getSquareButton()) {
        armState = ArmState.Idle;
      } else if (System.currentTimeMillis() - waitStartTime > Constants.wheelWaitTime) {
        armPiston.set(false);
        armState = ArmState.RetractingArm;
      }
      break;
    case RetractingArm:
      //if (armPiston.get() == false) {
        armState = ArmState.Idle;
      //}
      break;
 
    }
    System.out.println("arm state is " + armState);
    System.out.println("angle of servo is " + armPiston.get());
 
  }
 
  char getFMSColour = getFMSColour();
 
  public char getFMSColour() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      colourGoal = gameData.charAt(0);
      return colourGoal;
    } else {
      return 'n';
    }
  }
 
  char getColourFromSensor;
 
  private char getColourFromSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
    char result;
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      result = 'b';
    } else if (match.color == kRedTarget) {
      result = 'r';
    } else if (match.color == kGreenTarget) {
      result = 'g';
    } else if (match.color == kYellowTarget) {
      result = 'y';
    } else {
      result = 'n';
    }
    return result;
  }
 
 
 
 
}
 

