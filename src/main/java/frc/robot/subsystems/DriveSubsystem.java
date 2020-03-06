/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.components.PathParams;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.*;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  WPI_TalonSRX leftBackSide;
  WPI_TalonSRX leftFrontSide;
  WPI_TalonSRX rightFrontSide;
  WPI_TalonSRX rightBackSide;

  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide; 
  
  AHRS navX;

  final double targetControllerKp = 3.75; 
  final double targetControllerKi = 0.0;
  final double targetControllerKd = 0.0625;
  final double targetControllerPeriod = 1.0/50.0;
  PIDController targetController;
  
  final double powerCellKp = 0.375; 
  final double powerCellKi = 0.0;
  final double powerCellKd = 0.0;
  final double powerCellPeriod = 1.0/50.0;
  PIDController powerCellController; 
  
  private double originalAngleToTarget;
  private double lastCycleAngleToTarget;
  private double currentAngleToTarget;

  enum TeleopDriveState {
    Manual,
    AutoTargetAlign,
    Powercell,
    PathFollowing
  }

  TeleopDriveState teleopDriveState;

  NetworkTableInstance networkTableInst ;
  NetworkTableEntry canSeeTargetEntry ;
  NetworkTableEntry targetAngleEntry ;
  NetworkTableEntry targetDistanceEntry ;

  NetworkTableEntry canSeePowercellEntry ;
  NetworkTableEntry powercellAngleEntry ;
  NetworkTableEntry powercellDistanceEntry ;

  NetworkTableEntry autoAlignKpEntry;
  NetworkTableEntry autoAlignKiEntry;
  NetworkTableEntry autoAlignKdEntry;

  private boolean finishedAligning;
  private boolean finishedPath;
  private ArrayList<Double> lastThreeAngleErrors;
  private double alignStartTime;
  private int alignCycleCount;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveSubsystem(){
    leftFrontSide = new WPI_TalonSRX(Constants.ID_leftFrontMotor);
    leftBackSide = new WPI_TalonSRX(Constants.ID_leftBackMotor);
    rightFrontSide = new WPI_TalonSRX(Constants.ID_rightFrontMotor);
    rightBackSide = new WPI_TalonSRX(Constants.ID_rightBackMotor);

    navX = new AHRS(SerialPort.Port.kUSB);

    leftBackSide.setInverted(false);
    leftBackSide.setInverted(false);
    leftFrontSide.follow(leftBackSide);
    initializePIDConfig(leftBackSide);

    rightBackSide.setInverted(true);
    rightFrontSide.setInverted(true);
    rightFrontSide.follow(rightBackSide);
    initializePIDConfig(rightBackSide);

    targetController = new PIDController(targetControllerKp, targetControllerKi, targetControllerKd, targetControllerPeriod);
    targetController.setTolerance(Constants.angleThreshold);
    targetController.setSetpoint(0.0);

    powerCellController = new PIDController(powerCellKp, powerCellKi, powerCellKd, powerCellPeriod);
    powerCellController.setSetpoint(0.0);
  
    teleopDriveState = TeleopDriveState.Manual;

    networkTableInst = NetworkTableInstance.getDefault() ;

    canSeeTargetEntry = networkTableInst.getTable("vision").getEntry("canSeeTarget") ;
    targetAngleEntry = networkTableInst.getTable("vision").getEntry("targetAngle") ;
    targetDistanceEntry = networkTableInst.getTable("vision").getEntry("targetDistance") ;

    canSeePowercellEntry = networkTableInst.getTable("vision").getEntry("canSeePowercell") ;
    powercellAngleEntry = networkTableInst.getTable("vision").getEntry("powercellAngle") ;
    powercellDistanceEntry = networkTableInst.getTable("vision").getEntry("powercellDistance") ;

    autoAlignKpEntry = networkTableInst.getTable("shooter").getEntry("autoAlignKp");
    autoAlignKiEntry = networkTableInst.getTable("shooter").getEntry("autoAlignKi");
    autoAlignKdEntry = networkTableInst.getTable("shooter").getEntry("autoAlignKd");

    currentAngleToTarget = 0;
    lastCycleAngleToTarget = 0;

    finishedAligning = false;
    finishedPath = false;
    lastThreeAngleErrors = new ArrayList<Double>();
    alignStartTime = 0.0;
    alignCycleCount = 0;
  }



  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }


  public void teleopDrive(double leftJoy, double rightJoy, boolean targetAligning, boolean chaseCell, boolean driveOnPath, PathParams pathParams){
    SmartDashboard.putNumber("NavX", navX.getAngle());
    switch (teleopDriveState) {
      case Manual:
        if (targetAligning && canSeeTarget()) {
            targetController.reset();
            navX.reset();
            originalAngleToTarget = angleToTargetFromVision();
            teleopDriveState = TeleopDriveState.AutoTargetAlign;
            finishedAligning = false;
            alignStartTime = System.currentTimeMillis();
        } 
        else if (chaseCell && canSeePowercell()) {
          teleopDriveState = TeleopDriveState.Powercell ;
        }
        else if (driveOnPath){
          teleopDriveState = TeleopDriveState.PathFollowing;
        }
        else {
          arcadeDrive(leftJoy, rightJoy);
        }
        break;

      case AutoTargetAlign:
        if (!targetAligning || (lastThreeAngleErrors.size() >= Constants.alignMemorySize && angleRelativelyStatic()) || System.currentTimeMillis() - alignStartTime >= Constants.alignTimeoutTime){
          leftBackSide.set(0.0);
          rightBackSide.set(0.0);
          teleopDriveState = TeleopDriveState.Manual;
          finishedAligning = true;
        } 
        // else if(finishedAligningUncorrected()){
        //   originalAngleToTarget = angleToTargetFromVision();
        //   if(NavXAndVisionAgree()){
        //     teleopDriveState = TeleopDriveState.Manual;
        //     finishedAligning = true;
        //   }
        //   else
        //     currentAngleToTarget = angleToTargetFromVision();
        // }
        else {
            currentAngleToTarget = originalAngleToTarget - (navX.getAngle()*Constants.degreesToRadiansFactor);
            SmartDashboard.putNumber("NavX Value", navX.getAngle());
            double output = targetController.calculate(currentAngleToTarget);
            leftBackSide.set(ControlMode.Velocity, output * Constants.VelocityInputConversionFactor);
            rightBackSide.set(ControlMode.Velocity, -output * Constants.VelocityInputConversionFactor);
            lastCycleAngleToTarget = currentAngleToTarget;
            if(alignCycleCount >= 10){
              lastThreeAngleErrors.add(currentAngleToTarget);
            }
            alignCycleCount++;
            alignCycleCount %= 10;
        }
        break;

      case Powercell:
        if (!chaseCell || !canSeePowercell()){
          teleopDriveState = TeleopDriveState.Manual;
        } 
        else {
          double anglePowerCell = angleToPowercell() ;
          double output = powerCellController.calculate(anglePowerCell) ; 
          leftBackSide.set(ControlMode.Velocity, (output - Constants.speedConstantForBallChase) * Constants.VelocityInputConversionFactor);
          rightBackSide.set(ControlMode.Velocity, (output + Constants.speedConstantForBallChase) * Constants.VelocityInputConversionFactor);
        }
        break;
      case PathFollowing:
        leftBackSide.set(ControlMode.MotionMagic, pathParams.getLeftRotations());
        rightBackSide.set(ControlMode.MotionMagic, pathParams.getRightRotations());
        if(Math.abs(leftBackSide.getActiveTrajectoryPosition() - leftBackSide.getClosedLoopTarget()) < 20 
           && Math.abs(leftBackSide.getActiveTrajectoryPosition() - leftBackSide.getClosedLoopTarget()) < 20){
          finishedPath = true;
        }
      }
      autoAlignKpEntry = networkTableInst.getTable("").getEntry("autoAlignKp");
      autoAlignKiEntry = networkTableInst.getTable("").getEntry("autoAlignKi");
      autoAlignKdEntry = networkTableInst.getTable("").getEntry("autoAlignKd");
  }

  

  private boolean angleRelativelyStatic(){
    if(lastThreeAngleErrors.size() >= 3){
      return Math.abs(lastThreeAngleErrors.get(0) - lastThreeAngleErrors.get(2)) < Constants.maxAngleChangeForAlignFinish
          && Math.abs(lastThreeAngleErrors.get(1) - lastThreeAngleErrors.get(2)) < Constants.maxAngleChangeForAlignFinish
          && Math.abs(lastThreeAngleErrors.get(0) - lastThreeAngleErrors.get(1)) < Constants.maxAngleChangeForAlignFinish
          && Math.abs(lastThreeAngleErrors.get(2)) < 1.0;
    }
    else
      return false;
  }



  private boolean canSeeTarget(){
    return canSeeTargetEntry.getBoolean(false) ;
  }



  private boolean canSeePowercell(){
    return canSeePowercellEntry.getBoolean(false);
  }



  private double angleToTargetFromVision() {
    return targetAngleEntry.getDouble(0.0) ;
  }



  private double angleToPowercell() {
    return powercellAngleEntry.getDouble(0.0);
  }


  private boolean finishedAligningUncorrected(){
    return Math.abs(currentAngleToTarget - lastCycleAngleToTarget) < Constants.maxAngleChangeForAlignFinish;
  }


  private boolean NavXAndVisionAgree(){
    return Math.abs(originalAngleToTarget - currentAngleToTarget) < Constants.maxAngleDifferenceBetweenNavXAndVision;
  }



  private void initializePIDConfig(WPI_TalonSRX talon){
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime); //Encoder as feedback device, main PID loop, 30 ms timeout time
    talon.configClosedloopRamp(Constants.closedVoltageRampingConstant);
    talon.configOpenloopRamp(Constants.manualVoltageRampingConstant);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1.0);
    talon.configPeakOutputReverse(-1.0);
    talon.configMotionCruiseVelocity((int)(Constants.unitsPerRotation * Constants.desiredRPMsForDrive));
    talon.config_kF(Constants.PID_id, Constants.DrivetrainKf);
    talon.config_kP(Constants.PID_id, Constants.DrivetrainkP);
    talon.config_kI(Constants.PID_id, 0);
    talon.config_kD(Constants.PID_id, 0);
  }



  private void arcadeDrive(double speed, double angle){

    speed = -speed;

    double leftPower = 0;
    double rightPower = 0;

    angle *= 0.5;
    speed *= 0.5;

    leftPower = speed + angle;
    rightPower = speed - angle;

    SmartDashboard.putNumber("Left Power", leftPower);
    
    leftBackSide.set(ControlMode.Velocity, leftPower * Constants.maxDriveVelocity);
    rightBackSide.set(ControlMode.Velocity, rightPower * Constants.maxDriveVelocity);
  }



  public boolean finishedAligning(){
    return finishedAligning;
  }


  public void driveRight(){
    rightBackSide.set(0.5);
  }



  public boolean pathIsFinished(){
    return finishedPath;
  }
}
