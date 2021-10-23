package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.team1678.frc2021.SwerveModule;
import com.team1678.frc2021.lib.util.TimeDelayedBoolean;
import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret implements Subsystem {
    private TalonFX mMaster;

    Turret() {
        //there's only one TalonFX, so we can call just call it "Turret"
        mMaster = new TalonFX(Constants.turretMotorId);
        mMaster.set(ControlMode.PercentOutput, 0.62);

        //sets the software limits for the spin of the turret
        mMaster.RightSideLimit(true);
        mMaster.LeftSideLimit(true);
        mMaster.setRightSideLimit(Constants.MaxRadAngle / (235.0 * Constants.RotationsPerTick));
        mMaster.setReverseSoftLimit(Constants.MinRadAngle / (235.0 * Constants.RotationsPerTick));
    }

    //sets the turret to starting position, in our case, facing the left
    synchronized void SetsAngle(Rotation2d angle) {
        mMaster.changeControlMode(mMaster.TalonControlMode.Position);
        mMaster.set(angle.getRadians() / (2 * Math.PI * Constants.RotationPerTick));
    }

    @Override
    public void periodic() {
        // Runs every tick
    }

}
