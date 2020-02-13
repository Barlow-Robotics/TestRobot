/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.OI;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexingSubsystem;


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
  private int cycleIndex, cellCount;

  
  OI oi = new OI();

  //Encoder left = new Encoder(Constants.leftEncoderPorts[0], Constants.leftEncoderPorts[1]);

  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubystem;
  ArmSubsystem armSubsystem;
  ClimbSubsystem climbSubsystem;
  IndexingSubsystem indexingSubsystem;
  NetworkTableInstance networkTable;
  NetworkTableEntry frameTime;
  NetworkTableEntry loopTime;

  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //e = new Encoder(Constants.leftEncoderPorts[0], Constants.leftEncoderPorts[1]);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //======================
    cellCount = 0;
    //======================

    driveSubsystem = new DriveSubsystem();
    shooterSubystem = new ShooterSubsystem();
    armSubsystem = new ArmSubsystem();
    climbSubsystem = new ClimbSubsystem();
    indexingSubsystem = new IndexingSubsystem();

    networkTable = NetworkTableInstance.getDefault(); 
    frameTime = networkTable.getTable("performance").getEntry("frameTime");
    loopTime = networkTable.getTable("performance").getEntry("loopTime");

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
    //xhcjgvhkblj;kn
    SmartDashboard.putNumber("Angle To Target", driveSubsystem.getAngleToTarget());
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
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
    //Write to NetworkTables millis() - LastStart
    //LastStart = millis()
    previousStartTime = System.currentTimeMillis();

    //SHOOTER SUBSYSTEM
    shooterSubystem.operateShooterOP();

    //DRIVE SUBSYSTEM   
    driveSubsystem.teleopDrive(oi.getLeftY(), oi.getRightX());

    // ARM SUBSYSTEM
    armSubsystem.OperateControlPanel();

    //CLIMB SUBSYSTEM
    // if(oi.getRTopTrigger()) climbSubsystem.moveUp();
    // else if(oi.getRBottomTrigger()) climbSubsystem.moveDown();
    // else climbSubsystem.stopClimb();

    //INDEXING SUBSYSTEM
    indexingSubsystem.operateIndex();

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

  }

  public void incrementBallCount(){cellCount++;}
}