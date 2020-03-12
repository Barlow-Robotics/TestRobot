package frc.robot; 


public class Constants{
    //Motor ports
        public static final int ID_leftBackMotor = 1;
        public static final int ID_leftFrontMotor = 2;
        public static final int ID_rightFrontMotor = 3;
        public static final int ID_rightBackMotor = 4;
        public static final int ID_shooterMotor = 6;
        public static final int ID_shooterFeedMotor = 7;
        public static final int ID_climbMotor = 8;

        public static final int PWMPORT_agitatorMotor = 3;
        public static final int PWMPORT_intakeMotorPort = 1;
        public static final int PWMPORT_wheelSpinner = 2;

        public static final int SOLENOID_extendIntake = 3;
        public static final int SOLENOID_retractIntake = 2;

    //Sensors
        //Indexing ball count sensors
        public static final int DIOPORT_intakeSensor = 1;
        public static final int DIOPORT_intakeTransmitter = 0;
        public static final int DIOPORT_exitSensor = 3;
        public static final int DIOPORT_exitTransmitter = 2;


    //General
        public static final double degreesToRadiansFactor = Math.PI/180;
        public static final double wheelTurnsToDistanceFactor = 7.65 * Math.PI;

    //Drivetrain
        public static final double drivetrainMinPower = 0.05;
        public static final double drivetrainMaxPower = 1.0;
        public static final double manualVoltageRampingConstant = 1.0;
        public static final double closedVoltageRampingConstant = 1.0;

        public static final int PID_id = 0;
        public static final double PID_Period = 1.0/20.0;
        public static final double DrivetrainKf = 0.1797; //0.1797
        public static final double DrivetrainkP = 0.02;

        public static final double unitsPerRotation = 8192;
        public static final double RPMsToUnitsPerHundredMilliseconds = 1.0/600.0;
        public static final double desiredRPMsForDrive = 500.0;
        public static final double maxDriveVelocity = 6000.0;
        public static final double VelocityInputConversionFactor = desiredRPMsForDrive * unitsPerRotation * RPMsToUnitsPerHundredMilliseconds;


        public static final int timeoutTime = 30;
        public static final int mainFeedbackLoop = 0;

        public static final double autoBackingDistance = 3.5; //3.5 rotations of the wheel ~ 65"
        public static final double pathFollowingThreshold = 20;
        public static final int autonomousDriveTime = 2500;
        public static final double autonomousDriveSpeed = 0.7;
        public static final double autonomousTurnRate = 0.7;

        public static final double speedConstantForBallChase = 0.3;
        public static final double maxAngleChangeForAlignFinish = 0.5;
        public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
        public static final double alignTimeoutTime = 1000;
        public static final double alignMemorySize = 3;

        //Physical
            public static final double halfDistanceBetweenWheels = 10.75; //Inches

    //OI //hi :)
        public static final int joystickPort = 0;
        public static final int boxControlPort = 1;

    //Index
        public static final double desiredFeedingPercent = 0.75;
        public static final double agitatingSpeed = 0.35;
        public static final double maxFeedingVelocity = -34712;
        public static final double agitatorCyclePeriod = 1000;

        public static final double indexingkF = 0.028;
        public static final double indexingkP = 0.015;
        public static final double indexingkD = 0.003;

    //Intake
        public static final int intakeDeployPort = 2;
        public static final int intakeRetractPort = 3;
        public static final double intakeSpeed = 0.50;

    //Wheel
        public static final int openSolenoidDeployPort = 1;
        public static final int closeSolenoidDeployPort = 0;
        
        public static final double maxSpinSpeed = -0.35;
        public static final double minSpinSpeed = 0.02;
    
        public static final int colourFilterLength = 3;
	    public static final int minColorChangeCountGoalTurn= 26;
	    public static final int minColorChangeCountGoalSpecificColor= 4;
    
        public static final long wheelWaitTime = 5000;
        public static final double wheelDecrementFactor = 0.9;

        public static final char Blue = 'B';
        public static final char Red = 'R';
        public static final char Green = 'G';
        public static final char Yellow = 'Y';
        public static final char NullColorConstant = 'N';

    //Shooter
	    public static final double maxShooterSpeed = 47816; //Units per 100 milliseconds
	    public static final double desiredShooterPercent = 0.25; //CHANGE THIS
        public static final double minShooterSpeed = 0.05;        
        public static final double desiredSensorUnitsPerHundredMilliseconds = 47618;
        public static final double minShooterPercentForFiring = 0.9;
        public static final double shooterSpinupTimeout = 1000;

        public static final double shooterkF = 0.021;
        public static final double shooterkP = 0.1;
        public static final double shooterkD = 0.003;

        public static final int leftServoPort = 0;
        public static final int rightServoPort = 1;

        public static final double angleThreshold = 0.01;
        
        public static final double setAngleOfServo = 0.05;

    //Climb
        public static final int desiredClimberRotationsUp = 5;
        public static final int desiredClimberRotationsDown = 3;
        public static final double LiftMotorSpeed = 0.32 ;
        public static final int tolerableUnitsFromMaxClimberValue = 50;

}