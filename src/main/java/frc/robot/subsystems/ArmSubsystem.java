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
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  private int desiredNumberOfColourChanges;
 
  char currentColour = Constants.NullColorConstant;
  char lastColour = Constants.NullColorConstant;
  char FMSColour = Constants.NullColorConstant;

  private boolean previousInput;
 
  private final static I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch m_colorMatcher = new ColorMatch();
 
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
 
  //put calibrated colours here
 
  private Solenoid openSolenoidDeploy = new Solenoid(Constants.openSolenoidDeployPort);
  private Solenoid closeSolenoidDeploy = new Solenoid(Constants.closeSolenoidDeployPort);
  private boolean wheelDeployed, colorMatched;
  private WPI_TalonSRX wheelSpinner = new WPI_TalonSRX(1);
 
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
  int desiredNumberOfColorChanges;
 
 
  public ArmSubsystem() {
    armState = ArmState.Idle;
    spinningMode = SpinningMode.Which;
    colourFilter = new ColourFilter(Constants.colourFilterLength, 'n');
    previousInput = false;
    wheelDeployed = false;
    desiredNumberOfColorChanges = 0;

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
  }
 
  @Override
  public void initDefaultCommand() {
    wheelSpinner.set(0.0);
    deployWheel(false);
  }
 
  public void OperateControlPanel() {
    switch (armState) {
    case Idle:
      wheelSpinner.set(0.0);
      deployWheel(false);
      if (oi.isOperatingWheel()) {
        deployWheel(true);
        armState = ArmState.DeployingArm;
        previousInput = true;
      }
      break;
    case DeployingArm:
      if (previousInput != oi.isOperatingWheel() || colorMatched) {
        armState = ArmState.Idle;
      } 
      else {
        FMSColour = getFMSColour();
        previousInput = oi.isOperatingWheel();
        armState = ArmState.SpinningWheel;
        colourChangeCounter = 0;
        lastColour = colourFilter.getColour();
        if (FMSColour == Constants.NullColorConstant){
          desiredNumberOfColorChanges = Constants.minColorChangeCountGoal;
        } 
      }
      break;
    case SpinningWheel:
      FMSColour = getFMSColour();
      if (previousInput != oi.isOperatingWheel()) {
        armState = ArmState.Idle;
      } 
      else {
        previousInput = oi.isOperatingWheel();
        currentColour = colourFilter.getColour();
        if(currentColour == FMSColour)
          desiredNumberOfColorChanges = 2;
        else
          desiredNumberOfColorChanges = -1;
      }

      if(desiredNumberOfColourChanges > 0){
        lastColour = currentColour;
        currentColour = colourFilter.getColour();
        wheelSpinner.set(Constants.maxSpinSpeed);
        if(lastColour != currentColour)
          desiredNumberOfColourChanges--;
      }
      else if(desiredNumberOfColourChanges == -1){

      }
      else{
        wheelSpinner.set(0.0);
        armState = armState.WaitingForTimeout;
      }
      break;
    case WaitingForTimeout:
      if (!oi.isOperatingWheel() || colorMatched) {
        armState = ArmState.Idle;
      } 
      else if (System.currentTimeMillis() - waitStartTime > Constants.wheelWaitTime) {
        wheelDeployed = false;
        armState = ArmState.RetractingArm;
      }
      break;
    case RetractingArm:
      deployWheel(false);
      armState = ArmState.Idle;
      break;
    }
    // System.out.println("arm state is " + armState);
    // System.out.println("wheel is deployed: " + wheelDeployed);
  }
 
  

  private int calculateChangesToCorrectColor(char currentColour, char FMSColour){
    int changes = 0;
    if(currentColour == FMSColour)
       changes = 2;
    return changes;
  }



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


  
  private char getColourFromSensor() { 
    char result;
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      result = Constants.Blue;
    } else if (match.color == kRedTarget) {
      result = Constants.Red;
    } else if (match.color == kGreenTarget) {
      result = Constants.Green;
    } else if (match.color == kYellowTarget) {
      result = Constants.Yellow;
    } else {
      result = 'N';
    }
    return result;
  }


 
  private void deployWheel(boolean deploy){
    openSolenoidDeploy.set(deploy);
    closeSolenoidDeploy.set(!deploy);
  }
 
 
}
 

