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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

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
  
  AHRS navX;

  //public DifferentialDrive driveTrain;

  double leftPower, rightPower;
  final double targetControllerKp = 2.1; 
  final double targetControllerKi = 0.0;
  final double targetControllerKd = 0.4;
  final double targetControllerPeriod = 1.0/20.0;
  PIDController targetController;
  
  final double powerCellKp = 1.0; 
  final double powerCellKi = 0.0;
  final double powerCellKd = 0.2;
  final double powerCellPeriod = 1.0/20.0;
  PIDController powerCellController; 

//  final double powerSpeed = 0.5;
  final double powerSpeed = 0.0;
  
  private double originalAngleToTarget;

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
    rightFrontSide.follow(rightBackSide);

    leftBackSide.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30); //Encoder as feedback device, main PID loop, 30 ms timeout time
    rightBackSide.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30); //Encoder as feedback device, main PID loop, 30 ms timeout time

    targetController = new PIDController(targetControllerKp, targetControllerKi, targetControllerKd, targetControllerPeriod);
    targetController.setTolerance(Constants.angleThreshold);
    targetController.setSetpoint(0.0);

    powerCellController = new PIDController(powerCellKp, powerCellKi, powerCellKd, powerCellPeriod);
    powerCellController.setSetpoint(0.0);

    leftBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);
    rightBackSide.config_kF(Constants.PID_id, Constants.DrivetrainKf);

    //driveTrain = new DifferentialDrive(leftBackSide, rightBackSide);
  
    teleopDriveState = TeleopDriveState.Manual;
    autoDriveState = AutoDriveState.Backing;

    networkTableInst = NetworkTableInstance.getDefault() ;

    canSeeTargetEntry = networkTableInst.getTable("vision").getEntry("canSeeTarget") ;
    targetAngleEntry = networkTableInst.getTable("vision").getEntry("targetAngle") ;
    targetDistanceEntry = networkTableInst.getTable("vision").getEntry("targetDistance") ;

    canSeePowercellEntry = networkTableInst.getTable("vision").getEntry("canSeePowercell") ;
    powercellAngleEntry = networkTableInst.getTable("vision").getEntry("powercellAngle") ;
    powercellDistanceEntry = networkTableInst.getTable("vision").getEntry("powercellDistance") ;

  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new DriveCommand());
  }

  public void autonomousDrive(){

    switch(autoDriveState){
      case Backing:
        double targetPos = 4096 * Constants.autoBackingDistance;
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


  public void teleopDrive(double leftPower, double rightPower){
    SmartDashboard.putBoolean("Sees cell", canSeePowercell());
    switch (teleopDriveState) {
      case Manual:
        SmartDashboard.putBoolean("Can See Target", canSeeTarget());
        if (oi.getLTopTrigger() == true && canSeeTarget() == true) {
            System.out.println("entering auto align") ;
            targetController.reset();
            navX.reset();
            originalAngleToTarget = angleToTargetFromTables();
            teleopDriveState = TeleopDriveState.AutoTargetAlign;
        } else if ( oi.getRTopTrigger() && canSeePowercell() ) {
          teleopDriveState = TeleopDriveState.Powercell ;
        }  else {
          leftPower = threshold(leftPower);
          rightPower = threshold(rightPower);
          leftBackSide.set(ControlMode.Velocity, -leftPower * 500 * 4096 / 600);
          rightBackSide.set(ControlMode.Velocity, rightPower * 500 * 4096 / 600);
        }
        break;

      case AutoTargetAlign:
        if (oi.getLTopTrigger() == false /*|| canSeeTarget() == false*/){
          System.out.println("Exiting auto-align\n" + targetController.getPositionError()
                              + "\nAngle error from vision: " + angleToTargetFromTables());
          teleopDriveState = TeleopDriveState.Manual;
        } else {
            double angleToTarget = originalAngleToTarget - (navX.getAngle()*3.14159/180);
            SmartDashboard.putNumber("NavX Value", navX.getAngle());
            //if(angleToTarget < Constants.angleThreshold && abs(angleToTarget - angleToTargetFromTables) > Constants.angleThreshold) 
              //angleToTarget = angleToTargetFromTables();
            double output = targetController.calculate(angleToTarget);
            System.out.println( "angle to target is " + angleToTarget ) ;
            System.out.println( "output is " + output ) ;
            leftBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);
            rightBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);
              //driveTrain.tankDrive(-output, output);            
        }
        break;

      case Powercell:
        if (!oi.getRTopTrigger() ||  !canSeePowercell()){
          teleopDriveState = TeleopDriveState.Manual;
        } else {
         double anglePowerCell = angleToPowercell() ;
         System.out.println("angle to power cell is " + angleToPowercell()) ;

         double output = powerCellController.calculate(anglePowerCell) ; 
         System.out.println("output is " + output) ;
          //driveTrain.tankDrive(-output + powerSpeed, output + powerSpeed);
          leftBackSide.set(ControlMode.Velocity, (output - Constants.speedConstantForBallChase) * 500 * 4096 / 600);
          rightBackSide.set(ControlMode.Velocity, (output + Constants.speedConstantForBallChase) * 500 * 4096 / 600);

          // System.out.println( "angle to target is " + angleToTarget ) ;
          // System.out.println( "output is " + output ) ;
          // leftBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);
          // rightBackSide.set(ControlMode.Velocity, output * 500 * 4096 / 600);

        }
        break;
      }

  
  }

  public double getAngleToTarget(){ return angleToTargetFromTables();}
  

  private boolean canSeeTarget(){

    return canSeeTargetEntry.getBoolean(false) ;
    //return true ;
    // if (target == true){
    //   return true;
    // }
  }

  private boolean canSeePowercell(){
    return canSeePowercellEntry.getBoolean(false) ;
    // if (powerCell == true){
    //   return true;
    // }
    }

  private double angleToTargetFromTables() {
    return targetAngleEntry.getDouble(0.0) ;
  }

  private double angleToTargetFromEncoders(){
    return 0.0;
  }


 private double angleToPowercell() {
  return powercellAngleEntry.getDouble(0.0);
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

  // private void publishEncoderData(double left, double right){
  //   SmartDashboard.putBoolean("Outputting", true);
  //   SmartDashboard.putNumber("Left Data", left);
  //   SmartDashboard.putNumber("Right Data", right);
  // }

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
