/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.controller.PIDController;

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

  public DifferentialDrive driveTrain = new DifferentialDrive(leftSide, rightSide);

  double leftPower, rightPower;
  double[] encoderValues = new double[2];
  final double kp = 1.0 ;
  final double ki = 0.0;
  final double kd = 0.2;
  final double period = 1.0/20.0;
  PIDController targetController = new PIDController(kp, ki, kd, period);
  enum DriveState {
    Manual,
    AutoTargetAlign,
  }

  DriveState driveState ;


  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveSubsystem(){
    leftFrontSide = new WPI_TalonSRX(Constants.leftFrontMotor);
    leftBackSide = new WPI_TalonSRX(Constants.leftBackMotor);
    rightFrontSide = new WPI_TalonSRX(Constants.rightFrontMotor);
    rightBackSide = new WPI_TalonSRX(Constants.rightBackMotor);

    leftBackSide.follow(leftFrontSide);
    rightBackSide.follow(rightFrontSide);
    
    leftSide = new SpeedControllerGroup(leftFrontSide, leftBackSide); 
    rightSide = new SpeedControllerGroup(rightFrontSide, rightBackSide);

    leftSideEncoder = new Encoder(Constants.leftEncoderPorts[0], Constants.leftEncoderPorts[1]);
    leftSideEncoder.reset();

    rightSideEncoder = new Encoder(Constants.rightEncoderPorts[0], Constants.rightEncoderPorts[1]);
    rightSideEncoder.reset();
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }



  public void teleopDrive(double leftPower, double rightPower){

    switch (driveState) {
      case Manual:
        if (oi.getCircleButton() == true && canSeeTarget() == true) {
          driveState = DriveState.AutoTargetAlign;
        }
        break;

      case AutoTargetAlign:
        if (oi.getCircleButton() == true && canSeeTarget() == true){
          
        }
        else if (oi.getCircleButton() == false || canSeeTarget() == false){
          driveState = DriveState.Manual;
        }
        break;
      }
    leftPower = threshold(leftPower);
    rightPower = threshold(rightPower);
    
    publishEncoderData(leftSideEncoder.get(), rightSideEncoder.get());

    driveTrain.tankDrive(-leftPower, -rightPower);
  
  }
  
  private boolean canSeeTarget(){
    return true ;
    // if (target == true){
    //   return true;
    // }
  }

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
