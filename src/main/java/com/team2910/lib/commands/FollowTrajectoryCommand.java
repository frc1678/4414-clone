package com.team2910.lib.commands;

import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.control.Trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectoryCommand extends CommandBase {
    private final Swerve swerve;
    private final Trajectory trajectory;

    public FollowTrajectoryCommand(Swerve swerve, Trajectory trajectory) {
        this.swerve = swerve;
        this.trajectory = trajectory;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        //swerve.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        //swerve.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return true;
        //return swerve.getFollower().getCurrentTrajectory().isEmpty();
    }
}