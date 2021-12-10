// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2021;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.team1678.frc2021.autos.*;
import com.team1678.frc2021.commands.*;
import com.team1678.frc2021.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Button zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final Button yButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final Button bButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final Button aButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final Button xButton = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /* Autonomous Selector */
  private final AutonomousSelector autonomousSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    autonomousSelector = new AutonomousSelector();

    // Configure the button bindings
    configureButtonBindings();

  }

  public double getRotationAxis() {
    return driver.getRawAxis(rotationAxis);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // Snap Commands
    yButton.whenPressed(new InstantCommand(() -> s_Swerve.startSnap(0)));
    bButton.whenPressed(new InstantCommand(() -> s_Swerve.startSnap(90)));
    aButton.whenPressed(new InstantCommand(() -> s_Swerve.startSnap(180)));
    xButton.whenPressed(new InstantCommand(() -> s_Swerve.startSnap(270)));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousSelector.getCommand(s_Swerve);
  }

  public void zeroGyro() {
    s_Swerve.zeroGyro();
  }

  public void resetAngleToAbsolute() {
    s_Swerve.resetAngleToAbsolute();
  }
}
