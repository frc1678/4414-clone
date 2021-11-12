package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Turret;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DroppingIntakeCommand extends CommandBase {
    
    private final Superstructure mSuperstructure;
    private final double mTurretAngle;
    private double mTurretSetpoint = 0.0; 

    public DroppingIntakeCommand(Superstructure superstructure, double turretAngle) {
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
    }

    @Override
    public void initialize() {
        mSuperstructure.setWantDroppingIntake(Rotation2d.fromDegrees(mTurretAngle));

    }

    @Override
    public void execute() {
        mSuperstructure.setWantDroppingIntake(Rotation2d.fromDegrees(mTurretAngle));
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.isDroppingIntake && Math.abs(mSuperstructure.mTurret.getAngle.getDegrees() - 90);
    }
}