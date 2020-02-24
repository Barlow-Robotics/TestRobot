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
import java.util.HashMap;
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

import frc.robot.components.ColorSensor;
import frc.robot.components.ColourFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
 
public class ArmSubsystem extends Subsystem {
  char colourGoal;
 
  OI oi = new OI();
 
  char currentColour = Constants.NullColorConstant;
  char lastColour = Constants.NullColorConstant;
  char FMSColour = Constants.NullColorConstant;

  HashMap<Character, Character> FMSColourToDesiredColour;

  private boolean previousInput;
 
  ColorSensor colorSensor;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
 
  //put calibrated colours here
 
  private Solenoid openSolenoidDeploy;
  private Solenoid closeSolenoidDeploy;
  private boolean colorMatched;
  private Spark wheelSpinner = new Spark(0);
  
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
 
  public ArmSubsystem() {
    percentCompleteArm = 0.0;
    wheelPercentage.setNumber(percentCompleteArm);
    wheelPercentage = networkTableInst.getTable("WheelOfFortune").getEntry("wheelPercentage");
    armState = ArmState.Idle;
    colorSensor = new ColorSensor();
    colourFilter = new ColourFilter(Constants.colourFilterLength, 'n');
    previousInput = false;
    desiredNumberOfColorChanges = 0;
    desiredColor = 'N';
    colorForCalibration = 0;

    //Map colors under the wheel sensor to colors under *our* sensor
    FMSColourToDesiredColour = new HashMap<Character, Character>();
    FMSColourToDesiredColour.put('R', 'B'); //If the field wants red, we need blue under our sensor
    FMSColourToDesiredColour.put('B', 'R'); 
    FMSColourToDesiredColour.put('G', 'Y'); 
    FMSColourToDesiredColour.put('Y', 'G'); 

    openSolenoidDeploy = new Solenoid(Constants.openSolenoidDeployPort);
    closeSolenoidDeploy = new Solenoid(Constants.closeSolenoidDeployPort);

    networkTableInst = NetworkTableInstance.getDefault();
    wheelState = networkTableInst.getTable("WheelOfFortune").getEntry("wheelState");
  }


 
  @Override
  public void initDefaultCommand() {
  }

  
  
 
  public void OperateControlPanel() {
    wheelState.forceSetString(armState.toString());
    // SmartDashboard.putString("Arm State", armState.toString());
    switch (armState) {
    case Idle:
      wheelSpinner.set(0.0);
      deployWheel(false);
      if (oi.isOperatingWheel()) {
        percentCompleteArm = 0.0;
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
        desiredNumberOfColorChanges = 0;
        if (FMSColour == Constants.NullColorConstant){
          desiredNumberOfColorChanges = Constants.minColorChangeCountGoalTurn;
          colourChangeCountGoal = Constants.minColorChangeCountGoalTurn; }
        else{
          desiredColor = FMSColourToDesiredColour.get(FMSColour);
          desiredNumberOfColorChanges = Constants.minColorChangeCountGoalSpecificColor;
          colourChangeCountGoal = Constants.minColorChangeCountGoalSpecificColor;
        }
        System.out.println(desiredColor);
        armState = ArmState.SpinningWheel;
      }
      break;
      
    case SpinningWheel:
      wheelSpinner.set(Constants.maxSpinSpeed);
      currentColour = getColourFromSensor();
      if(previousInput != oi.isOperatingWheel() && oi.isOperatingWheel()){
        System.out.println("OP opt-out");
        armState = ArmState.SpinningDown;
      }
      else if(currentColour == desiredColor){
        System.out.println("Control exit");
        wheelSpinner.set(0.0);
        armState= ArmState.SpinningDown;
      }
      else if(desiredNumberOfColorChanges <= 0 && FMSColour == Constants.NullColorConstant){
        System.out.println("Color Change number exit");
        wheelSpinner.set(0.0);
        armState= ArmState.SpinningDown;
      }
      else{
        if(currentColour != lastColour && FMSColour == Constants.NullColorConstant){
          desiredNumberOfColorChanges--;
          System.out.println("Change " + desiredNumberOfColorChanges);
        }
      }
      lastColour = currentColour;
      percentCompleteArm = 0.6 * (colourChangeCountGoal -desiredNumberOfColorChanges ) ;

      break;
    case SpinningDown:
    wheelSpinner.set(wheelSpinner.get()*Constants.wheelDecrementFactor);
      if(Math.abs(wheelSpinner.get()) <= Constants.minSpinSpeed){
        wheelSpinner.set(0);
        armState = ArmState.WaitingForTimeout;
        waitStartTime = System.currentTimeMillis();
      }
    break;
    case WaitingForTimeout:
      timeElapsed = System.currentTimeMillis() - waitStartTime;
      if (!oi.isOperatingWheel()) {
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
      deployWheel(false);
      armState = ArmState.Idle;
      break;
    }

    wheelPercentage.setNumber(percentCompleteArm);
    wheelPercentage = networkTableInst.getTable("WheelOfFortune").getEntry("wheelPercentage");
  }


  public void sendState(){
    wheelState.setString(armState.toString());
  }


  public void setStateToIdle(){
    armState = ArmState.Idle;
  }


  public void calibrationMode(){
    SmartDashboard.putString("Calibrate", "Put color "+ colorSensor.getColorAtIndex(colorForCalibration) + " under sensor, then press the pink button");
    if(oi.getPinkButton()){
      colorSensor.calibrateColor(colorForCalibration);
      colorForCalibration++;
    }
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