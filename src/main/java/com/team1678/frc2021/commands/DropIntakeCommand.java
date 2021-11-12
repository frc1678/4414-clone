package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Turret;
import com.team1678.frc2021.subsystem.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DropIntakeCommand extends CommandBase {
    private final mTurret = turret; 
    private final mSuperstructure = superstructure;
}

@Override
public void initialize() {
    //set turret to 90 degrees 
}

@Override
public boolean isFinished() {
    //return Turret.getInstance().getStart
}
