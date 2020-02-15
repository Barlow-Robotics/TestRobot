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
import edu.wpi.first.wpilibj.Compressor;
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
 
  char currentColour = 'a';
  char lastColour = 'b';

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
  private Compressor compressor = new Compressor(0);
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
 
 
  public ArmSubsystem() {
    armState = ArmState.Idle;
    colourFilter = new ColourFilter(Constants.colourFilterLength, 'n');
    previousInput = false;
    wheelDeployed = false;
    spinningMode = SpinningMode.Which;
  }
 
  @Override
  public void initDefaultCommand() {
    wheelSpinner.set(0.0);
    wheelDeployed = false;
    updatePistons();
  }
 
  public void OperateControlPanel() {
    switch (armState) {
    case Idle:
      wheelSpinner.set(0.0);
      wheelDeployed = false;
      updatePistons();
      if (oi.getSquareButton()) { //what button is used ?? MUST CHANGE
        wheelDeployed = true;
        updatePistons();
        armState = ArmState.DeployingArm;
        System.out.println("arm state is " + armState);
        System.out.println("wheel is deployed: " + wheelDeployed);
    
        previousInput = true;
      }
      break;
    case DeployingArm:
      if (!oi.getSquareButton() || colorMatched) {
        armState = ArmState.Idle;
        System.out.println("arm state is " + armState);
        System.out.println("wheel is deployed: " + wheelDeployed);
          } else  {
        previousInput = oi.getSquareButton();
        armState = ArmState.SpinningWheel;
        colourChangeCounter = 0;
        lastColour = colourFilter.getColour();
        wheelSpinner.set(Constants.maxSpinSpeed);
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
              System.out.println("State: " + spinningMode);
            } 
            else{
              spinningMode = SpinningMode.TurnColour;
              System.out.println("State: " + spinningMode);

            }
            break;
          case TurnTimes:

            colourFilter.addMeasurement(getColourFromSensor());
            currentColour = colourFilter.getColour();
            if (currentColour != lastColour) {
              System.out.println("Change:"+colourChangeCounter);
              colourChangeCounter++;
              lastColour = currentColour;
              System.out.println("Color changed ("+colourChangeCounter+")");
            }
            if (colourChangeCounter > Constants.minColorChangeCountGoal) {
              waitStartTime = System.currentTimeMillis();
              wheelSpinner.set(0.0);
              armState = ArmState.WaitingForTimeout;
              System.out.println("arm state is " + armState);
              System.out.println("wheel is deployed: " + wheelDeployed);
            }
            break;
          case TurnColour:
            currentColour = getColourFromSensor();
            if (currentColour == getFMSColour()){
                colorMatched = true;
            }
            else if(colorMatched){
              System.out.println("Matched");
              System.out.println("Number of Changes:" + colourChangeCounter);
              if (colourChangeCounter < 2){
                if (currentColour != lastColour) {
                  lastColour = currentColour;
                  colourChangeCounter++;
                }  
              }
              else{
                wheelSpinner.set(0.0);
                armState = ArmState.WaitingForTimeout;
                System.out.println("arm state is " + armState);
                System.out.println("wheel is deployed: " + wheelDeployed);
              }  
            }
            else{
               wheelSpinner.set(Constants.maxSpinSpeed);
            }
           
            break;
        }
      }
      break;
    case WaitingForTimeout:
      if (!oi.getSquareButton() || colorMatched) {
        armState = ArmState.Idle;
        System.out.println("arm state is " + armState);
        System.out.println("wheel is deployed: " + wheelDeployed);
          } else if (System.currentTimeMillis() - waitStartTime > Constants.wheelWaitTime) {
        wheelDeployed = false;
        // updatePistons();
        armState = ArmState.RetractingArm;
      }
      break;
    case RetractingArm:
      wheelDeployed = false;
      updatePistons();
      armState = ArmState.Idle;
      System.out.println("arm state is " + armState);
      System.out.println("wheel is deployed: " + wheelDeployed);
        break;
 
    }
    // System.out.println("arm state is " + armState);
    // System.out.println("wheel is deployed: " + wheelDeployed);
 
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
      result = 'B';
    } else if (match.color == kRedTarget) {
      result = 'R';
    } else if (match.color == kGreenTarget) {
      result = 'g';
    } else if (match.color == kYellowTarget) {
      result = 'Y';
    } else {
      result = 'N';
    }
    return result;
  }
 
  private void updatePistons(){
    openSolenoidDeploy.set(wheelDeployed);
    closeSolenoidDeploy.set(!wheelDeployed);
  }
 
 
}
 

