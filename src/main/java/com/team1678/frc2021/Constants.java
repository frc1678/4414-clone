package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.team1678.frc2021.subsystems.LinearServo;
import com.team1678.frc2021.subsystems.ServoSubsystem;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    // Control Board
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

    public static final class Swerve {
        public static final int pigeonID = 23;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(1.0);
        public static final double wheelBase = Units.inchesToMeters(1.0);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.86;
        public static final double angleGearRatio = 12.8;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 17;
            public static final double angleOffset = 69;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final double angleOffset = 108;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 19;
            public static final double angleOffset = 358;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 20;
            public static final double angleOffset = 107;
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

    //Hood
    public static final ServoSubsystem.ServoSubsystemConstants kHoodConstants = new ServoSubsystem.ServoSubsystemConstants();
    static {
        kHoodConstants.kHoodServoA = 2;
        kHoodConstants.kHoodServoB = 3;
        kHoodConstants.kHoodSpeed = 1;

        kHoodConstants.kHoodRadius = 11.904; // radius of hood
        kHoodConstants.kAbsoluteEncoderID = 3;
    }

    public static final double kHoodMinLimit = 15; //TODO: check value

    //Intake
    public static int masterIntakeMotorId = 7;

    //Shooter
    public static final int kMasterFlywheelID = 20;
    public static final int kSlaveFlywheelID = 21;
    public static final double kShooterP = 0.2;
    public static final double kShooterI = 0.00004;
    public static final double kShooterD = 0.0;
    public static final double kShooterF = 0.05;

    //Turret
    public static int turretMotorId = 10;
    public static int MaxRadAngle = 2;
    public static int MinRadAngle = 2;
    public static double RotationsPerTick;
    public static int kTurretConstants;

    /* Hopper constants */
    public static final int masterElevatorMotorId = 69; // TODO: Update can ID
    public static final int slaveElevatorMotorId = 70; // TODO: Update can ID
    public static final int elevatorSensorPin = 69; // TODO: Update pin #
    public static final double elevatingDemand = 7;
    public static final double hellavatingDemand = 9;

    }

