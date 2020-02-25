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

  //Encoder left = new Encoder(Constants.leftEncoderPorts[0], Constants.leftEncoderPorts[1]);

  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;

  // ArmSubsystem armSubsystem;
  ColorSensor colorSensor;

  ClimbSubsystem climbSubsystem;
  IndexingSubsystem indexingSubsystem;
  IntakeSubsystem intakeSubsystem;

  NetworkTableInstance networkTable;
  NetworkTableEntry frameTime;
  NetworkTableEntry loopTime;
  NetworkTableEntry wheelState;

  // Compressor compressor;
  

  public enum AutoState{
    Idle, 
    DrivingPath, 
    Searching, 
    Aligning, 
    Firing
  };

  AutoState autoState;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //e = new Encoder(Constants.leftEncoderPorts[0], Constants.leftEncoderPorts[1]);
    networkTable = NetworkTableInstance.getDefault(); 

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // driveSubsystem = new DriveSubsystem();
    shooterSubsystem = new ShooterSubsystem(networkTable);

    // armSubsystem = new ArmSubsystem();

    // climbSubsystem = new ClimbSubsystem();
    indexingSubsystem = new IndexingSubsystem(networkTable);
    // intakeSubsystem = new IntakeSubsystem();

    autoState = AutoState.Idle;

    
    frameTime = networkTable.getTable("performance").getEntry("frameTime");
    loopTime = networkTable.getTable("performance").getEntry("loopTime");
    wheelState = networkTable.getTable("WheelOfFortune").getEntry("wheelState");

    // compressor = new Compressor(1);
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
    // armSubsystem.setStateToIdle();
  }

  @Override
  public void disabledPeriodic() {
    // armSubsystem.sendCurrentColour();
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
    autoState = AutoState.DrivingPath;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch(autoState){
      case Idle:
        shooterSubsystem.operateShooter(true);
        indexingSubsystem.operateIndex(false, false);
        driveSubsystem.teleopDrive(0, 0, false, false, false, null);
      break;
      case DrivingPath:
        shooterSubsystem.operateShooter(true);
        // driveSubsystem.teleopDrive(0, 0, false, false, true, new PathParams());
        if(driveSubsystem.pathIsFinished())
          autoState = AutoState.Aligning;
      break;
      case Aligning:
        shooterSubsystem.operateShooter(true);
        driveSubsystem.teleopDrive(0, 0, true, false, false, null);
        if(driveSubsystem.finishedAligning())
          autoState = AutoState.Firing;
      break;
      case Firing:
        shooterSubsystem.operateShooter(true);
        indexingSubsystem.operateIndex(true, false);
        if(indexingSubsystem.getCellCount() == 0)
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    startTime = System.currentTimeMillis();

    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();

    // intakeSubsystem.operateIntake(oi.getDeployIntakeManual());
    indexingSubsystem.operateIndex(oi.getIsShooting(), false);
    shooterSubsystem.operateShooter(true);

    endTime = System.currentTimeMillis();
    duration = endTime - startTime;

    frameTime.forceSetNumber(duration);
    //Send data to NetworkTable "performance" with entry "frameTime" for every cycle
    //Send lastTime difference to entry "loopTime"
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    indexingSubsystem.operateIndex(oi.getIsShooting(), false);
    shooterSubsystem.operateShooter(true);
  }
}