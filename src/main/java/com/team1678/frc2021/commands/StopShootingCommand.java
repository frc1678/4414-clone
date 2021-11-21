package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopShootingCommand extends CommandBase{
    
    private final Superstructure mSuperstructure;

    public StopShootingCommand (Superstructure superstructure) {

        mSuperstructure = superstructure;
    }

    @Override 
    public void initialize() {
        mSuperstructure.setWantShoot(false);
    }

    @Override
    public void execute() {
        mSuperstructure.setWantShoot(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
