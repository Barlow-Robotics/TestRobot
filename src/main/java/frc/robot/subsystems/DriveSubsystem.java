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
import frc.robot.OI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  OI oi = new OI();
  WPI_TalonSRX leftBackSide;
  WPI_TalonSRX leftFrontSide;
  WPI_TalonSRX rightFrontSide;
  WPI_TalonSRX rightBackSide;

  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide; 
  
  AHRS navX;

  final double targetControllerKp = 1.05; 
  final double targetControllerKi = 0.0;
  final double targetControllerKd = 0.2;
  final double targetControllerPeriod = 1.0/20.0;
  PIDController targetController;
  
  final double powerCellKp = 0.375; 
  final double powerCellKi = 0.0;
  final double powerCellKd = 0.0;
  final double powerCellPeriod = 1.0/20.0;
  PIDController powerCellController; 

  final double powerSpeed = 0.0;
  
  private double originalAngleToTarget;
  private double currentAngleToTarget;

  enum TeleopDriveState {
    Manual,
    AutoTargetAlign,
    Powercell
  }

  enum AutoDriveState{
    Backing,
    SearchingForTarget,
    Firing,
    TargetingMoreCells, //Optimistically
    ChasingCells
  }

  TeleopDriveState teleopDriveState;
  AutoDriveState autoDriveState;

  NetworkTableInstance networkTableInst ;
  NetworkTableEntry canSeeTargetEntry ;
  NetworkTableEntry targetAngleEntry ;
  NetworkTableEntry targetDistanceEntry ;

  NetworkTableEntry canSeePowercellEntry ;
  NetworkTableEntry powercellAngleEntry ;
  NetworkTableEntry powercellDistanceEntry ;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveSubsystem(){
    leftFrontSide = new WPI_TalonSRX(Constants.leftFrontMotor);
    leftBackSide = new WPI_TalonSRX(Constants.leftBackMotor);
    rightFrontSide = new WPI_TalonSRX(Constants.rightFrontMotor);
    rightBackSide = new WPI_TalonSRX(Constants.rightBackMotor);

    navX = new AHRS(SerialPort.Port.kUSB);

    leftFrontSide.follow(leftBackSide);
    leftBackSide.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime); //Encoder as feedback device, main PID loop, 30 ms timeout time
    leftBackSide.configClosedloopRamp(Constants.voltageRampingConstant);
    leftBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);

    rightFrontSide.follow(rightBackSide);
    rightBackSide.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.mainFeedbackLoop, Constants.timeoutTime); //Encoder as feedback device, main PID loop, 30 ms timeout time
    rightBackSide.configClosedloopRamp(Constants.voltageRampingConstant);
    rightBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);

    targetController = new PIDController(targetControllerKp, targetControllerKi, targetControllerKd, targetControllerPeriod);
    targetController.setTolerance(Constants.angleThreshold);
    targetController.setSetpoint(0.0);

    powerCellController = new PIDController(powerCellKp, powerCellKi, powerCellKd, powerCellPeriod);
    powerCellController.setSetpoint(0.0);
  
    teleopDriveState = TeleopDriveState.Manual;
    autoDriveState = AutoDriveState.Backing;

    networkTableInst = NetworkTableInstance.getDefault() ;

    canSeeTargetEntry = networkTableInst.getTable("vision").getEntry("canSeeTarget") ;
    targetAngleEntry = networkTableInst.getTable("vision").getEntry("targetAngle") ;
    targetDistanceEntry = networkTableInst.getTable("vision").getEntry("targetDistance") ;

    canSeePowercellEntry = networkTableInst.getTable("vision").getEntry("canSeePowercell") ;
    powercellAngleEntry = networkTableInst.getTable("vision").getEntry("powercellAngle") ;
    powercellDistanceEntry = networkTableInst.getTable("vision").getEntry("powercellDistance") ;

    currentAngleToTarget = 0;
  }



  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }



  public void autonomousDrive(){
    switch(autoDriveState){
      case Backing:
        double targetPos = Constants.unitsPerRotation * Constants.autoBackingDistance;
        leftBackSide.set(ControlMode.MotionMagic, targetPos);
        rightBackSide.set(ControlMode.MotionMagic, targetPos);
        break;
      case SearchingForTarget:

      break;

      case Firing:

      break;

      case TargetingMoreCells:

      break;

      case ChasingCells:

      break;

      default:
    }
  }



  public void teleopDrive(double robotSpeed, double robotAngle, boolean aligning){
    SmartDashboard.putBoolean("Sees cell", canSeePowercell());
    switch (teleopDriveState) {
      case Manual:
        SmartDashboard.putBoolean("Can See Target", canSeeTarget());
        if (aligning == true && canSeeTarget() == true) {
            targetController.reset();
            navX.reset();
            originalAngleToTarget = angleToTargetFromVision();
            teleopDriveState = TeleopDriveState.AutoTargetAlign;
        } 
        else if ( oi.isBallChasing() && canSeePowercell() ) {
          teleopDriveState = TeleopDriveState.Powercell ;
        }  
        else {
          arcadeDrive(robotSpeed, robotAngle);
        }
        break;

      case AutoTargetAlign:
        if (oi.isAutoTargeting() == false /*|| canSeeTarget() == false*/){
          teleopDriveState = TeleopDriveState.Manual;
        } else {
            currentAngleToTarget = originalAngleToTarget - (navX.getAngle()*Constants.degreesToRadiansFactor);
            SmartDashboard.putNumber("NavX Value", navX.getAngle());
            double output = targetController.calculate(currentAngleToTarget);
            leftBackSide.set(ControlMode.Velocity, output * Constants.VelocityInputConversionFactor);
            rightBackSide.set(ControlMode.Velocity, output * Constants.VelocityInputConversionFactor);
        }
        break;

      case Powercell:
        if (!oi.isBallChasing() ||  !canSeePowercell()){
          teleopDriveState = TeleopDriveState.Manual;
        } 
        else {
          double anglePowerCell = angleToPowercell() ;
          double output = powerCellController.calculate(anglePowerCell) ; 
          leftBackSide.set(ControlMode.Velocity, (output - Constants.speedConstantForBallChase) * Constants.VelocityInputConversionFactor);
          rightBackSide.set(ControlMode.Velocity, (output + Constants.speedConstantForBallChase) * Constants.VelocityInputConversionFactor);
        }
        break;
      }
  }

  

  public boolean getIsAligned(){
    return currentAngleToTarget < Constants.angleThreshold;
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



  private void arcadeDrive(double speed, double angle){

    double leftPower = 0;
    double rightPower = 0;

    //Figure out which input is stronger, adjust sign accordingly
    double defaultInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(angle)), speed);

    if(speed >= Constants.drivetrainMinPower){
      //Right-forward, else left-forward
      if(angle >= Constants.drivetrainMinPower){
        leftPower = defaultInput;
        rightPower = speed - angle;
      }
      else if(angle <= Constants.drivetrainMinPower){
        leftPower = speed - angle;
        rightPower = defaultInput;
      }
    }
    else if(speed <= -Constants.drivetrainMinPower){
      //Left-backward, else right-backward
      if(angle >= Constants.drivetrainMinPower){
        leftPower = speed + angle;
        rightPower = defaultInput;
      }
      else if(angle <= Constants.drivetrainMinPower){
        leftPower = defaultInput;
        rightPower = speed - angle;
      }
    }
    else {
      leftPower = 0;
      rightPower = 0;
    }
    //Set motor output
    leftBackSide.set(ControlMode.Velocity, leftPower * 500 * Constants.unitsPerRotation / 600);
    rightBackSide.set(ControlMode.Velocity, rightPower * 500 * Constants.unitsPerRotation / 600);
  }
}
