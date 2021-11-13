package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.team1678.frc2021.subsystems.LinearServo;
import com.team1678.frc2021.subsystems.ServoSubsystem;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import com.team254.lib.geometry.*;

import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.lib.util.SwerveModuleConstants;
import com.team1678.frc2021.subsystems.Limelight.LimelightConstants;

public final class Constants {
    public static final int kLongCANTimeoutMs = 100;
    
    public static final double stickDeadband = 0.1;

    // Control Board
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

    public static final class Swerve {
        public static final int pigeonID = 23;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(16.0);
        public static final double wheelBase = Units.inchesToMeters(16.0);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.86;
        public static final double angleGearRatio = 12.8;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.05;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.5;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = true;

        /* Module Specific Constants */

        /* Front Right Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final double angleOffset = 252;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 17;
            public static final double angleOffset = 292;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }



        /* Back Right Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 20;
            public static final double angleOffset = 251;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Back Left Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 19;
            public static final double angleOffset = 2.3;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class SnapConstants {
        public static final double snapKP = 5;
        public static final double snapKI = 0;
        public static final double snapKD = 0;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

        //Constraints for the profiled angle controller (vals stolen from AutoConstants)
        public static final double kMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final double kSlowMaxSpeedMetersPerSecond = 2.0;
        public static final double kSlowMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //Servo
    public static final int kServo = 7;
    public static final double kServoExtend = 1.0;
    public static final double kServoRetract = 0.0;

   // Hood Constants
   public static final int kServoAChannel = 8;
   public static final int kServoBChannel = 9;
   public static final double kHoodMinLimit = 0.5; //TODO: check value
   public static final double kHoodMaxLimit = 4.5;

    //Intake
    public static int masterIntakeMotorId = 7;

    //Shooter
    public static final int kMasterFlywheelID = 20;
    public static final int kOverheadFlywheelID = 21;

    public static final double kShooterFlywheelP = 0.07;
    public static final double kShooterFlywheelI = 0.0;
    public static final double kShooterFlywheelD = 0.0;
    public static final double kShooterFlywheelF = 0.073;

    public static final double kShooterOverheadP = 0.05;
    public static final double kShooterOverheadI = 0.0;
    public static final double kShooterOverheadD = 0.0;
    public static final double kShooterOverheadF = 0.053;

    //Turret
    public static int kTurretID = 10;
    public static int kTurretEncoderID = 1;
    public static int kTurretEncoderOffset = 0;
    public static final boolean kTurretInvertMotor = false;
	public static final double kTurretGearRatio = (2048.0 * 27.84) / 360.0;
	public static final double kTurretP = 0.05;
	public static final double kTurretI = 0.0;
	public static final double kTurretD = 0.0;
	public static final double kTurretF = 0.05;
	public static final double kTurretMinLimit = -130.0; // TODO: Check value with absolute encoder
	public static final double kTurretMaxLimit = 100.0; // TODO: Check value with absolute encoder
	public static final double kTurretCruiseVelocity = 10000;
	public static final double kTurretCruiseAcceleration = 10000;
	public static final double kTurretDeadband = 0;

    /* Hopper constants */
    public static final int hopperMotorId = 11; // TODO: Update can ID
    public static final int elevatorMotorId = 1; // TODO: Update can ID
    public static final int elevatorSensorPin = 69; // TODO: Update pin #
    public static final double elevatingDemand = 7;
    public static final double hellavatingDemand = 9;

    public static final double kLooperDt = 0;
    
    // Climber
    public static final int motorClimberID = 22;
    public static final int kLongCANTimeouts = 100;

    // limelight
	 public static final LimelightConstants kLimelightConstants = new LimelightConstants();
	 static {
		 kLimelightConstants.kName = "Limelight";
		 kLimelightConstants.kTableName = "limelight";
		 kLimelightConstants.kHeight = 24.5; // inches
		 kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromRadians(34.0);
     }
     
     public static final double TARGET_HEIGHT = 98.25;
     public static final double LIMELIGHT_HEIGHT = 24.1;
     public static final double kGoalHeight = 90.0;
	 public static final double kInnerGoalDepth = 29.25;
	 public static final double kInnerGoalToApex = 16.92;
     public static final double kInnerTargetRangeAngle = Math.toRadians(10.0); 
     public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
     public static final double kHorizontalFOV = 59.6; // degrees
	 public static final double kVerticalFOV = 49.7; // degrees
	 public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
	 public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

    }

