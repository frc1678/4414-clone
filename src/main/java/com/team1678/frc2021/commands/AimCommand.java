package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final Turret mTurret;
    private final double mTurretSetpoint;

    public AimCommand(Superstructure superstructure, Turret turret, double turretSetpoint) {
        mSuperstructure = superstructure;
        mTurret = turret;
        mTurretSetpoint = turretSetpoint;
    }

    @Override
    public void execute() {
        mTurret.setSetpointMotionMagic(mTurretSetpoint);
        mSuperstructure.setmWantVisionAim(true);
    }

    //mTurret.setMotionMagic


    
}
