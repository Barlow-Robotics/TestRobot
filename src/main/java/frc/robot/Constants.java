package frc.robot;


public class Constants{
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
    public static final double DrivetrainKf = 0.1797; //0.1797
    public static final int unitsPerRotation = 8096;

    public static final double autoBackingDistance = 3.5; //3.5 rotations of the wheel ~ 65"

    public static final double speedConstantForBallChase = 0.5;

    //OI //hi :)
    public static final int joystickPort = 0;
    public static final int boxControlPort = 1;

    //Wheel
    public static final int wheelRotationSparkPort = 0;
    public static final int wheelDeploymentSparkPort = 1;

    public static final double maxDeploySpeed = 0.65;
    public static final double maxSpinSpeed = 0.2;

    public static final double deployAngle = 90.0;
    public static final int minColorChangeCountGoal= 26 ;

    public static final long wheelWaitTime = 5000;


    //Shooter
    public static final double maxShooterSpeed = 0.9;
    public static final double minShooterSpeed = 0.05;

    public static final int shooterMotor = 1;

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