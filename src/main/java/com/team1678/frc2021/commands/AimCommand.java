package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final double mTurretSetpoint;

    public AimCommand(Superstructure superstructure, double turretSetpoint) {
        mSuperstructure = superstructure;
        mTurretSetpoint = turretSetpoint;
    }


    @Override
    public void initialize() {
        mSuperstructure.setmWantVisionAim(true);
        mSuperstructure.setWantTurretHint(true, mTurretSetpoint);
    }

    @Override
    public void execute() {
        mSuperstructure.setmWantVisionAim(true);
    }

    
    @Override
    public boolean isFinished() {
        return true;
    }


    
}
