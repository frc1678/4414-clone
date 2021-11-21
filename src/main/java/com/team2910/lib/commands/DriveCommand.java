package com.team2910.lib.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.math.Vector2;
import com.team2910.lib.robot.Axis;

public class DriveCommand extends CommandBase {
    private Swerve drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public DriveCommand(Swerve drivetrain, Axis forward, Axis strafe, Axis rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(new Translation2d(forward.get(true), strafe.get(true)), rotation.get(true)*0.5, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        //drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
        //drivetrainSubsystem.disableRotationMotors();
        //drivetrainSubsystem.setModuleCoastMode();
    }
}