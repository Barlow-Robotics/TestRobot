/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.HashMap;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;

import frc.robot.components.ColorSensor;
import frc.robot.components.ColourFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
 
public class ArmSubsystem extends Subsystem {
  char colourGoal;
  
  char currentColour = Constants.NullColorConstant;
  char lastColour = Constants.NullColorConstant;
  char FMSColour = Constants.NullColorConstant;

  HashMap<Character, Character> FMSColourToDesiredColour;

  private boolean previousInput;
 
  ColorSensor colorSensor;

  private Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  private Solenoid openSolenoidDeploy;
  private Solenoid closeSolenoidDeploy;
  private Spark wheelSpinner;
  
  NetworkTableInstance networkTableInst;
  NetworkTableEntry wheelState;
  NetworkTableEntry timeOut;
  NetworkTableEntry wheelPercentage;

  long timeElapsed;

  int colourTurn;
 
  ColourFilter colourFilter;
  int colorForCalibration; //G, B, Y, R
 
  double percentCompleteArm;
 
  long waitStartTime;
 
  enum ArmState {
    Idle, 
    DeployingArm, 
    SpinningWheel, 
    SpinningDown, 
    WaitingForTimeout, 
    RetractingArm
  };
 
  ArmState armState;
 
  int desiredNumberOfColorChanges;
  char desiredColor;
 
  int colourChangeCountGoal;
 
  public ArmSubsystem(NetworkTableInstance networkTableInst) {
    this.networkTableInst = networkTableInst;
    percentCompleteArm = 0.0;
    wheelPercentage = networkTableInst.getTable("WheelOfFortune").getEntry("wheelPercentage");
    wheelPercentage.setNumber(percentCompleteArm);
    armState = ArmState.Idle;
    colorSensor = new ColorSensor();
    colourFilter = new ColourFilter(Constants.colourFilterLength, 'n');
    previousInput = false;
    desiredNumberOfColorChanges = 0;
    desiredColor = 'N';
    colorForCalibration = 0;
    lastColour = 'N';

    //Map colors under the wheel sensor to colors under *our* sensor
    FMSColourToDesiredColour = new HashMap<Character, Character>();
    FMSColourToDesiredColour.put('R', 'B'); //If the field wants red, we need blue under our sensor
    FMSColourToDesiredColour.put('B', 'R'); 
    FMSColourToDesiredColour.put('G', 'Y'); 
    FMSColourToDesiredColour.put('Y', 'G'); 

    openSolenoidDeploy = new Solenoid(Constants.openSolenoidDeployPort);
    closeSolenoidDeploy = new Solenoid(Constants.closeSolenoidDeployPort);

    wheelSpinner = new Spark(Constants.PWMPORT_wheelSpinner);

    networkTableInst = NetworkTableInstance.getDefault();
    wheelState = networkTableInst.getTable("WheelOfFortune").getEntry("wheelState");
    timeOut = networkTableInst.getTable("WheelOfFortune").getEntry("timeOut");
  }


 
  @Override
  public void initDefaultCommand() {
  }

  
  
 
  public void OperateControlPanel(boolean operate) {
    wheelState.forceSetString(armState.toString());
    // SmartDashboard.putString("Arm State", armState.toString());
    switch (armState) {
    case Idle:
      wheelSpinner.set(0.0);
      deployWheel(false);
      if (operate) {
        percentCompleteArm = 0.0;
        deployWheel(true);
        armState = ArmState.DeployingArm;
        previousInput = true;
      }
      break;
    case DeployingArm:
        FMSColour = getFMSColour();
        desiredNumberOfColorChanges = 0;
        if (FMSColour == Constants.NullColorConstant){
          desiredNumberOfColorChanges = Constants.minColorChangeCountGoalTurn;
          colourChangeCountGoal = Constants.minColorChangeCountGoalTurn; }
        else{
          desiredColor = FMSColourToDesiredColour.get(FMSColour);
          desiredNumberOfColorChanges = Constants.minColorChangeCountGoalSpecificColor;
          colourChangeCountGoal = Constants.minColorChangeCountGoalSpecificColor;
        }
        armState = ArmState.SpinningWheel;
      break;
      
    case SpinningWheel:
      SmartDashboard.putNumber("Color Changes Left", desiredNumberOfColorChanges);
      SmartDashboard.putString("Current Color: ", getColourFromSensor() + " ");
      wheelSpinner.set(Constants.maxSpinSpeed);
      currentColour = getColourFromSensor();
      if(previousInput != operate && operate){
        armState = ArmState.Idle;
      }
      else if(currentColour == desiredColor){
        System.out.println("Control exit");
        armState = ArmState.SpinningDown;
      }
      else if(desiredNumberOfColorChanges <= 0 && FMSColour == Constants.NullColorConstant){
        System.out.println("Color Change number exit");
        armState= ArmState.SpinningDown;
      }
      else{
        if(currentColour != lastColour && FMSColour == Constants.NullColorConstant){
          desiredNumberOfColorChanges--;
          System.out.println("Change " + desiredNumberOfColorChanges);
        }
      }
      lastColour = currentColour;
      percentCompleteArm = 0.6 * (colourChangeCountGoal - desiredNumberOfColorChanges );
      break;
    case SpinningDown:
      if(operate != previousInput && operate){
        armState = ArmState.Idle;
      }
      wheelSpinner.set(wheelSpinner.get()*Constants.wheelDecrementFactor);
      if(Math.abs(wheelSpinner.get()) <= Constants.minSpinSpeed){
        wheelSpinner.set(0);
        armState = ArmState.WaitingForTimeout;
        waitStartTime = System.currentTimeMillis();
      }
    break;
    case WaitingForTimeout:
      timeElapsed = System.currentTimeMillis() - waitStartTime;
      if(operate != previousInput && operate){
        armState = ArmState.Idle;
      }
      else if (timeElapsed > Constants.wheelWaitTime) {
        armState = ArmState.RetractingArm;
      }
      timeOut.setNumber((double)(Constants.wheelWaitTime-timeElapsed)/1000.0);
      timeOut = networkTableInst.getTable("WheelOfFortune").getEntry("timeOut");
      percentCompleteArm = .6 + (double)((Constants.wheelWaitTime-timeElapsed)/1000.0) *0.4;
      break;
    case RetractingArm:
      wheelSpinner.set(0.0);
      // deployWheel(false);
      armState = ArmState.Idle;
      break;
    }
    previousInput = operate;
    wheelPercentage.setNumber(percentCompleteArm);
    wheelPercentage = networkTableInst.getTable("WheelOfFortune").getEntry("wheelPercentage");
  }



  public void sendState(){
    wheelState.setString(armState.toString());
  }


  public void stopWheel(){
    wheelSpinner.set(0.0);
  }



  public void setStateToIdle(){
    armState = ArmState.Idle;
  }


  boolean previousCalibrationInput = false;


  public void calibrationMode(boolean buttonPressed){
    SmartDashboard.putString("Calibrate", "Put color "+ colorSensor.getColorAtIndex(colorForCalibration) + " under sensor, then press the pink button");
    if(buttonPressed != previousCalibrationInput && previousCalibrationInput){
      colorSensor.calibrateColor(colorForCalibration);
      colorForCalibration++;
      colorForCalibration %= 4;
      SmartDashboard.putString("Calibrated Color!", " " + colorSensor.getCurrentColor());
    }
    previousCalibrationInput = buttonPressed;
  }



  public void sendCurrentColour(){
    String color = getColourFromSensor() + " ";
    SmartDashboard.putString("Color", color);
    SmartDashboard.putNumber("Red", colorSensor.getRed());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());
  }



  public char getFMSColour() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      colourGoal = gameData.charAt(0);
      return colourGoal;
    } else {
      return Constants.NullColorConstant;
    }
  }


  
  private char getColourFromSensor() { 
    char result;
    Color match = colorSensor.getCurrentColor();
    if (match == kBlueTarget) {
      result = Constants.Blue;
    } else if (match == kRedTarget) {
      result = Constants.Red;
    } else if (match == kGreenTarget) {
      result = Constants.Green;
    } else if (match == kYellowTarget) {
      result = Constants.Yellow;
    } else {
      result = Constants.NullColorConstant;
    }
    return result;
  }


 
  private void deployWheel(boolean deploy){
    openSolenoidDeploy.set(deploy);
    closeSolenoidDeploy.set(!deploy);
  }
 
 
}
 

/*FMSColour = getFMSColour();
      if (previousInput != oi.isOperatingWheel()) {
        armState = ArmState.Idle;
      } 
      else {
        previousInput = oi.isOperatingWheel();
        currentColour = colourFilter.getColour();
        if(currentColour == FMSColour && desiredNumberOfColorChanges == -1)
          desiredNumberOfColorChanges = 2;
        else
          desiredNumberOfColorChanges = -1;
      }

      currentColour = colourFilter.getColour();

      if(desiredNumberOfColorChanges > 0){
        lastColour = currentColour;
        wheelSpinner.set(Constants.maxSpinSpeed);
        if(lastColour != currentColour)
          desiredNumberOfColorChanges--;
      }
      else if(desiredNumberOfColorChanges == -1){
        if(currentColour == FMSColour)
          desiredNumberOfColorChanges = 2;
      }
      else{
        wheelSpinner.set(0.0);
        armState = armState.WaitingForTimeout;
      }*/