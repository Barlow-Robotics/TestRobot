package frc.robot;


public class Constants{
    //Drivetrain
    public static final int leftFrontMotor = 2;
    public static final int leftBackMotor = 1;
    public static final int rightFrontMotor = 3;
    public static final int rightBackMotor = 4;

    public static final double drivetrainMinPower = 0.05;
    public static final double drivetrainMaxPower = 0.8;

    public static final int[] leftEncoderPorts = {2, 3};
    public static final int[] rightEncoderPorts = {0, 1};

    public static final int PID_id = 0;
    public static final double DrivetrainKf = 0.1797; //0.1797

    //OI //hi :)
    public static final int joystickPort = 0;

    //Wheel
    public static final int wheelRotationSparkPort = 0;
    public static final int wheelDeploymentSparkPort = 1;

    public static final double maxDeploySpeed = 0.65;
    public static final double maxSpinSpeed = 0.5;

    //Shooter
    public static final double maxShooterSpeed = 0.9;
    public static final double minShooterSpeed = 0.05;

    public static final int shooterMotor = 1;

}