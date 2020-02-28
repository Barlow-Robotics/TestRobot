/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.components.PathParams;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  final double targetControllerKp = 1.0; 
  final double targetControllerKi = 0.0;
  final double targetControllerKd = 0.4;
  final double targetControllerPeriod = 1.0/50.0;
  PIDController targetController;
  
  final double powerCellKp = 0.375; 
  final double powerCellKi = 0.0;
  final double powerCellKd = 0.0;
  final double powerCellPeriod = 1.0/20.0;
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

  private boolean finishedAligning;
  private boolean finishedPath;

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

    currentAngleToTarget = 0;
    lastCycleAngleToTarget = 0;

    finishedAligning = false;
    finishedPath = false;
  }



  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }


  public void teleopDrive(double leftJoy, double rightJoy, boolean targetAligning, boolean chaseCell, boolean driveOnPath, PathParams pathParams){
    switch (teleopDriveState) {
      case Manual:
        if (targetAligning && canSeeTarget()) {
            targetController.reset();
            navX.reset();
            originalAngleToTarget = angleToTargetFromVision();
            teleopDriveState = TeleopDriveState.AutoTargetAlign;
            finishedAligning = false;
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
        if (!targetAligning){
          teleopDriveState = TeleopDriveState.Manual;
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
    talon.configClosedloopRamp(Constants.voltageRampingConstant);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1.0);
    talon.configPeakOutputReverse(-1.0);
    talon.configMotionCruiseVelocity((int)(Constants.unitsPerRotation * Constants.desiredRPMs));
    talon.config_kF(Constants.PID_id, Constants.DrivetrainKf);
    talon.config_kP(Constants.PID_id, Constants.DrivetrainkP);
    talon.config_kI(Constants.PID_id, 0);
    talon.config_kD(Constants.PID_id, 0);
  }



  private void arcadeDrive(double speed, double angle){

    speed = -speed;

    System.out.println("Speed: " + speed);
    System.out.println("Angle: "+angle);

    double leftPower = 0;
    double rightPower = 0;

    angle *= 0.3;

    leftPower = speed + angle;
    rightPower = speed - angle;

    System.out.println("Left power: " + leftPower);
    System.out.println("Right power: " + rightPower);


    leftBackSide.set(leftPower);
    rightBackSide.set(rightPower);
    // leftBackSide.set(ControlMode.Velocity, leftPower * 500 * Constants.unitsPerRotation / 600);
    // rightBackSide.set(ControlMode.Velocity, rightPower * 500 * Constants.unitsPerRotation / 600);
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
