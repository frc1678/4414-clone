package com.team1678.frc2021.autos;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import com.team1678.frc2021.subsystems.Swerve;

public class AutonomousSelector {

    private static SendableChooser<Rotation2d> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");
        orientationChooser = new SendableChooser<>();
        orientationChooser.setDefaultOption("Forward", Rotation2d.fromDegrees(0.0));
        orientationChooser.addOption("Backwards", Rotation2d.fromDegrees(180.0));
        orientationChooser.addOption("Left", Rotation2d.fromDegrees(90.0));
        orientationChooser.addOption("Right", Rotation2d.fromDegrees(270.0));
        autoTab.add("Starting Orientation", orientationChooser);

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("Test Straight", AutonomousMode.TEST_STRAIGHT_PATH);
        autoTab.add("Mode", autonomousModeChooser);
        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case TEST_STRAIGHT_PATH:
                return new TestStraightPath(s_Swerve);
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    public AutonomousSelector() {
    }

    private enum AutonomousMode {
        TEST_STRAIGHT_PATH,
    }

}
