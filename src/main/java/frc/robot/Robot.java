/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.OI;
import frc.robot.components.ColorSensor;
import frc.robot.components.PathParams;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private long startTime, endTime, duration, totalDuration, previousStartTime;

  public OI oi = new OI();

  // Encoder left = new Encoder(Constants.leftEncoderPorts[0],
  // Constants.leftEncoderPorts[1]);

  DriveSubsystem driveSubsystem;
  AHRS navX;
  ShooterSubsystem shooterSubsystem;

  ArmSubsystem armSubsystem;
  ColorSensor colorSensor;

  ClimbSubsystem climbSubsystem;
  IndexingSubsystem indexingSubsystem;
  IntakeSubsystem intakeSubsystem;

  NetworkInterface networkInterface;
  NetworkTableInstance networkTable;
  NetworkTableEntry frameTime;
  NetworkTableEntry loopTime;
  NetworkTableEntry wheelState;
  NetworkTableEntry IP_Address;

  NetworkTableEntry startOnFarSide; // This is all data for the robot's autonomous mode
  NetworkTableEntry startOnMiddle;
  NetworkTableEntry startOnCloseSide;

  byte[] ethernetByteList;
  Enumeration<NetworkInterface> allNetworkInterfaces;

  private double driveStartTime;
  private double autonomousStartTime ;



  // Compressor compressor;
  PowerDistributionPanel PDP;

  public enum AutoState {
    Idle, DrivingPath, DrivingFromLine, Searching, Aligning, Firing, Turn180
  };

  public enum StartingPosition {
    Close, Middle, Far
  };

  AutoState autoState;

  PathParams autoDriveParams;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    // Initialize our overall NetworkTable to connect with the Jetson
    networkTable = NetworkTableInstance.getDefault();

    navX = new AHRS(SerialPort.Port.kUSB);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveSubsystem = new DriveSubsystem(navX);
    shooterSubsystem = new ShooterSubsystem(networkTable);

    armSubsystem = new ArmSubsystem(networkTable);
    // climbSubsystem = new ClimbSubsystem();
    indexingSubsystem = new IndexingSubsystem(networkTable, 3);
    intakeSubsystem = new IntakeSubsystem();

    autoState = AutoState.Aligning;
    
    frameTime = networkTable.getTable("performance").getEntry("frameTime");
    loopTime = networkTable.getTable("performance").getEntry("loopTime");
    wheelState = networkTable.getTable("WheelOfFortune").getEntry("wheelState");

    startOnFarSide = networkTable.getTable("autoData").getEntry("startOnFarSide");
    startOnMiddle = networkTable.getTable("autoData").getEntry("startOnMiddle");
    startOnCloseSide = networkTable.getTable("autoData").getEntry("startOnCloseSide");

    // PDP = new PowerDistributionPanel();
    // PDP.clearStickyFaults();

    // compressor = new Compressor();
    // compressor.clearAllPCMStickyFaults();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  @Override
  public void disabledInit() {
    // indexingSubsystem.setSensorValues(false);
    armSubsystem.stopWheel();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //Owen | Change these values during testing to reflect a proper drive path
    if(startOnCloseSide.getBoolean(false)){
      autoDriveParams = new PathParams(0.0, 0.0);
    }
    else if(startOnMiddle.getBoolean(false)){
      autoDriveParams = new PathParams(0.0, 0.0);
    }
    else if(startOnFarSide.getBoolean(false)){
      autoDriveParams = new PathParams(0.0, 0.0);
    }
    else{
      autoDriveParams = new PathParams(0.0, 0.0);
    }

    autonomousStartTime = System.currentTimeMillis() ;
    autoState = AutoState.Aligning;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch(autoState){
      case Idle:
        //shooterSubsystem.operateShooter(true, false);
        indexingSubsystem.operateIndex(false, false, false, false);
        driveSubsystem.teleopDrive(0, 0, false, false, false, null);
      break;
      case Aligning:
        //shooterSubsystem.operateShooter(true, false);
        driveSubsystem.teleopDrive(0, 0, true, false, false, null);
        if(driveSubsystem.finishedAligning())
          autoState = AutoState.Firing;
      break;
      case Firing:
        //shooterSubsystem.operateShooter(true, false);
        indexingSubsystem.operateIndex(true, false, false, false);
        //wpk magic number alert!
        if(indexingSubsystem.getCellCount() == 0
           || System.currentTimeMillis() - autonomousStartTime > 10000) {
          // autoState = AutoState.Idle;
          autoState = AutoState.DrivingPath;
        }
      break;
      case DrivingPath:
        indexingSubsystem.operateIndex(false, false, false, false);
        this.driveStartTime = System.currentTimeMillis() ;
        //shooterSubsystem.operateShooter(true, false);
        driveSubsystem.teleopDrive(Constants.autonomousDriveSpeed, 0.0, false, false, false, null);
        autoState = AutoState.DrivingFromLine ;
        // if(driveSubsystem.pathIsFinished())
        //   autoState = AutoState.Idle;
      case DrivingFromLine:
          if (System.currentTimeMillis() - driveStartTime > Constants.autonomousDriveTime) {
             navX.reset();
             driveSubsystem.teleopDrive(0.0, Constants.autonomousTurnRate, false, false, false, null);
             autoState = AutoState.Turn180 ;
          }
      break;
      case Turn180:
        double currentAngle = navX.getAngle();
        if(Math.abs(currentAngle) < 180)
          driveSubsystem.teleopDrive(0.0, Constants.autonomousTurnRate, false, false, false, null);
        else
          autoState = AutoState.Idle;
      break;
      default:
        autoState = AutoState.Idle;
      break;
    }
    /*switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }*/
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    System.out.println("Period: " + this.m_period);
    previousStartTime = System.currentTimeMillis();
    System.out.println(ethernetByteList);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    startTime = System.currentTimeMillis();

    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();
    //============================================================
    driveSubsystem.teleopDrive(oi.getForwardSpeed(), oi.getTurnAngle(), oi.isAutoTargeting(), oi.isBallChasing(), false, null);
    indexingSubsystem.operateIndex(oi.getIsShooting(), oi.getDeployIntakeManual(), false, oi.getRedButton());
    //wpk add button check to folloinwg line
    intakeSubsystem.operateIntake(oi.getDeployIntakeManual() || oi.isBallChasing(), oi.getTriangleButton(), oi.getSquareButton()); 
    armSubsystem.OperateControlPanel(oi.getPinkButton());
    //============================================================
    endTime = System.currentTimeMillis();
    duration = endTime - startTime;

    frameTime.forceSetNumber(duration);
  }

  PathParams testParams;

  @Override
  public void testInit() {
    testParams = new PathParams(1.0, 12);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}