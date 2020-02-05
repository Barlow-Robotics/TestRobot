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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.controller.PIDController;

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

  Encoder leftSideEncoder;
  Encoder rightSideEncoder;

  public DifferentialDrive driveTrain;

  double leftPower, rightPower;
  double[] encoderValues = new double[2];
  final double targetControllerKp = 4.0; 
  final double targetControllerKi = 0.0;
  final double targetControllerKd = 0.0;
  final double targetControllerPeriod = 1.0/20.0;
  PIDController targetController;
  
  final double powerCellKp = 1.0; 
  final double powerCellKi = 0.0;
  final double powerCellKd = 0.2;
  final double powerCellPeriod = 1.0/20.0;
  PIDController powerCellController; 

  final double powerSpeed = 0.5;

  enum DriveState {
    Manual,
    AutoTargetAlign,
    Powercell
  }

  DriveState driveState;

  NetworkTableInstance networkTableInst ;
  NetworkTableEntry canSeeTargetEntry ;
  NetworkTableEntry targetAngleEntry ;
  NetworkTableEntry targetDistanceEntry ;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveSubsystem(){
    leftFrontSide = new WPI_TalonSRX(Constants.leftFrontMotor);
    leftBackSide = new WPI_TalonSRX(Constants.leftBackMotor);
    rightFrontSide = new WPI_TalonSRX(Constants.rightFrontMotor);
    rightBackSide = new WPI_TalonSRX(Constants.rightBackMotor);

    leftFrontSide.follow(leftBackSide);
    rightFrontSide.follow(rightBackSide);

    targetController = new PIDController(targetControllerKp, targetControllerKi, targetControllerKd, targetControllerPeriod);
    targetController.setSetpoint(0.0);

    powerCellController = new PIDController(powerCellKp, powerCellKi, powerCellKd, powerCellPeriod);
    powerCellController.setSetpoint(0.0);

    leftBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);
    rightBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);


    driveTrain = new DifferentialDrive(leftBackSide, rightBackSide);
  
    driveState = DriveState.Manual;

    networkTableInst = NetworkTableInstance.getDefault() ;

    canSeeTargetEntry = networkTableInst.getTable("vision").getEntry("canSeeTarget") ;
    targetAngleEntry = networkTableInst.getTable("vision").getEntry("targetAngle") ;
    targetDistanceEntry = networkTableInst.getTable("vision").getEntry("targetDistance") ;

  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }



  public void teleopDrive(double leftPower, double rightPower){
    switch (driveState) {
      case Manual:
        if (oi.getCircleButton() == true && canSeeTarget() == true) {
            targetController.reset();
            driveState = DriveState.AutoTargetAlign;
            System.out.println("entering auto align") ;
        } 
        else {
          leftPower = threshold(leftPower);
          rightPower = threshold(rightPower);
          leftBackSide.set(ControlMode.Velocity, -leftPower * 500 * 4096 / 600);
          rightBackSide.set(ControlMode.Velocity, rightPower * 500 * 4096 / 600);
          SmartDashboard.putNumber("Controller Input", leftPower);
        }
        break;

      case AutoTargetAlign:
        if (oi.getCircleButton() == false || canSeeTarget() == false){
            driveState = DriveState.Manual;
        } else {
           double angleToTarget = angleToTarget() ;
           double output = targetController.calculate(angleToTarget) ;
           System.out.println( "angle to target is " + angleToTarget ) ;
           System.out.println( "output is " + output ) ;
           leftBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);
           rightBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);
            //driveTrain.tankDrive(-output, output);            
        }
        break;

      case Powercell:
        if (oi.getSquareButton() == false || canSeePowerCell() == false){
          driveState = DriveState.Manual;
        } else {
         double anglePowerCell = angleToPowercell() ;
         double output = powerCellController.calculate(anglePowerCell) ; 
          driveTrain.tankDrive(-output + powerSpeed, output + powerSpeed);
        }
        break;
      }
  
  }
  

  private boolean canSeeTarget(){

    return canSeeTargetEntry.getBoolean(false) ;
    //return true ;
    // if (target == true){
    //   return true;
    // }
  }

  private boolean canSeePowerCell(){
    return true ; 
    // if (powerCell == true){
    //   return true;
    // }
    }

  private double angleToTarget() {
    return targetAngleEntry.getDouble(0.0) ;
    //return 0.0;  //fill in params!
 }


 private double angleToPowercell() {
  return 0.0;  //fill in params!
}

  //private double angleToPowerCell() {
    //return findAngle(); //fill in params!
  //}  

  // public double findAngle(double FOV, double screenWidth, int targetX) {
  //   double normalizedTargetX = (1/(screenWidth/2))*((double) targetX - ((screenWidth/2)-0.5));
  //   double viewPlaneWidth = 2.0*(Math.tan(FOV/2));
  //   double x = (viewPlaneWidth/2)*normalizedTargetX;
  //   return Math.atan(x);
  // }

  private void publishEncoderData(double left, double right){
    SmartDashboard.putBoolean("Outputting", true);
    SmartDashboard.putNumber("Left Data", left);
    SmartDashboard.putNumber("Right Data", right);
    SmartDashboard.putBoolean("Left Stopped", leftSideEncoder.getStopped());
  }

  private double threshold(double power){
    if(power < -Constants.drivetrainMaxPower)
      power = -Constants.drivetrainMaxPower;
    else if(power > Constants.drivetrainMaxPower)
      power = Constants.drivetrainMaxPower;
    else if(power < Constants.drivetrainMinPower)
      power = 0;
    else if(power > -Constants.drivetrainMinPower)
      power = 0;
    
    return power;
  }
}
