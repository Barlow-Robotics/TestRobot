package frc.robot;


public class Constants{
    //General
    public static final double degreesToRadiansFactor = Math.PI/180;

    //Drivetrain
    public static final int leftFrontMotor = 2;
    public static final int leftBackMotor = 1;
    public static final int rightFrontMotor = 3;
    public static final int rightBackMotor = 4;

    public static final double drivetrainMinPower = 0.05;
    public static final double drivetrainMaxPower = 1.0;
    public static final double voltageRampingConstant = 0.3;

    public static final int PID_id = 0;
    public static final double PID_Period = 1.0/20.0;
    public static final double DrivetrainKf = 0.1797; //0.1797

    public static final int unitsPerRotation = 8192;
    public static final int desiredRPMs = 500;
    public static final double RPMsToUnitsPerHundredMilliseconds = 1.0/600.0;
    public static final double VelocityInputConversionFactor = desiredRPMs * unitsPerRotation * RPMsToUnitsPerHundredMilliseconds;

    public static final int timeoutTime = 30;
    public static final int mainFeedbackLoop = 0;

    public static final double autoBackingDistance = 3.5; //3.5 rotations of the wheel ~ 65"

    public static final double speedConstantForBallChase = 0.5;

    //OI //hi :)
    public static final int joystickPort = 0;
    public static final int boxControlPort = 1;

    //Index
    public static final int indexingWheelMotor = 7;
    // public static final int agitatingWheelMotor = 7;

    public static final double feedingSpeed = 0.4;
    public static final double agitatingSpeed = 0.5;

    //Intake
    public static final int intakeMotorPort = 2;
    public static final int intakeDeployPort = 2;
    public static final int intakeRetractPort = 3;
    public static final double intakeSpeed = 0.4;

    //Wheel
    public static final int wheelRotationTalonID = 7;
    public static final int openSolenoidDeployPort = 0;
    public static final int closeSolenoidDeployPort = 1;
    
    public static final double maxSpinSpeed = -0.25;
 
    public static final int colourFilterLength = 3;
    public static final int minColorChangeCountGoal= 26;
 
    public static final long wheelWaitTime = 5000;
    public static final double wheelDecrementFactor = 0.02;

    public static final char Blue = 'B';
    public static final char Red = 'R';
    public static final char Green = 'G';
    public static final char Yellow = 'Y';
    public static final char NullColorConstant = 'N';


    //Shooter
    public static final double maxShooterSpeed = 0.5 * 500 * 8096 / 600;
    public static final double maxShooterPercent = 0.85;
    public static final double minShooterSpeed = 0.05;

    public static final int shooterMotorTalonID = 6;

    public static final int leftServoPort = 0;
    public static final int rightServoPort = 1;

    public static final double angleThreshold = 0.01;
    
    public static final double setAngleOfServo = 0.05;

    //Climb
    public static final int ClimbMotorPortNumber = 7;
    public static final int ShooterMotor1 = 6;
    public static final int desiredClimberRotationsUp = 5;
    public static final int desiredClimberRotationsDown = 3;
    public static final double LiftMotorSpeed = 0.32 ;
    public static final int tolerableUnitsFromMaxClimberValue = 50;

}