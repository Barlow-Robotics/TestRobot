package frc.robot;


public class Constants{
    //General


    //Drivetrain
    public static final int leftFrontMotor = 2;
    public static final int leftBackMotor = 1;
    public static final int rightFrontMotor = 3;
    public static final int rightBackMotor = 4;

    public static final double drivetrainMinPower = 0.05;
    public static final double drivetrainMaxPower = 1.0;

    public static final int[] leftEncoderPorts = {2, 3};
    public static final int[] rightEncoderPorts = {0, 1};

    public static final int PID_id = 0;
    public static final double PID_Period = 1.0/20.0;
    public static final double DrivetrainKf = 0.1797; //0.1797
    public static final int unitsPerRotation = 8096;

    public static final double autoBackingDistance = 3.5; //3.5 rotations of the wheel ~ 65"

    public static final double speedConstantForBallChase = 0.5;

    //OI //hi :)
    public static final int joystickPort = 0;
    public static final int boxControlPort = 1;

    //Index
    public static final int indexingWheelMotor = 7;

    public static final double feedingSpeed = 0.5;
    public static final double agitatingSpeed = 0.5;

    //Intake
    public static final int intakeDeployPort = 2;
    public static final int intakeRetractPort = 3;

    //Wheel
    public static final int wheelRotationTalonID = 7;
    public static final int openSolenoidDeployPort = 0;
    public static final int closeSolenoidDeployPort = 1;
    
    public static final double maxSpinSpeed = 0.25;
 
    public static final int colourFilterLength = 3;
    public static final int minColorChangeCountGoal= 26;
 
    public static final long wheelWaitTime = 5000;


    //Shooter
    public static final double maxShooterSpeed = 0.9 * 500 * 8096 / 600;
    public static final double minShooterSpeed = 0.05;

    public static final int shooterMotorTalonID = 6;

    public static final double angleThreshold = 0.01;
    
    public static final double setAngleOfServo = 0.0;

    //Climb
    public static final int ClimbMotorPortNumber = 7;
    public static final int ShooterMotor1 = 6;
    public static final int LiftEncoderChannelA = 0 ;
    public static final int LiftEncoderChannelB = 1 ;
    public static final int MaxEncoderValue = 1000;
    public static final double LiftMotorSpeed = 0.32 ;

}