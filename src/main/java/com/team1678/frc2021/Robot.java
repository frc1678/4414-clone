// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2021;

import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.loops.Looper;
import com.team1678.frc2021.subsystems.Climber;
import com.team1678.frc2021.subsystems.Hood;
import com.team1678.frc2021.subsystems.Hopper;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Turret;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  // private final Superstructure mSuperstructure = Superstructure.getInstance();
  // private final Hood mHood = Hood.getInstance();
  private final Hopper mHopper = Hopper.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  // private final Turret mTurret = Turret.getInstance();

    // loopers
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

  private boolean climbMode = false;
    

  public Robot() {

  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    mSubsystemManager.setSubsystems(
       mHopper,
       mIntake,
       mClimber,
       mShooter
       // mTurret
    ); 

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mSubsystemManager.registerDisabledLoops(mDisabledLooper);

    //TODO: figure out how to add hood


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    mSubsystemManager.outputToSmartDashboard();
    mEnabledLooper.outputToSmartDashboard();
    
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Controller Rotation",m_robotContainer.getRotationAxis());
    SmartDashboard.putBoolean("Intake Command", mControlBoard.getIntake());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mEnabledLooper.stop();
    mDisabledLooper.start();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    mDisabledLooper.stop();
    mEnabledLooper.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    mDisabledLooper.stop();
    mEnabledLooper.start();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (mControlBoard.climbMode()) {
      climbMode = true;
    }


    if (!climbMode) {
      if (mControlBoard.getIntake()) {
        mIntake.setState(Intake.WantedAction.INTAKE);
  
      } else if (mControlBoard.getShoot()) {
        mIntake.setState(Intake.WantedAction.INTAKE);
        mShooter.setVelocity(10);
        mHopper.setState(Hopper.WantedAction.ELEVATE);
  
     } //else if (mControlBoard.getReverseHopper()) {
  
      //     mHopper.setState(Hopper.WantedAction.REVERSE);
  
      // } else if (mControlBoard.getTurnOffIntake()) {
  
      //     mIntake.setState(Intake.WantedAction.NONE);
      //     mHopper.setState(Hopper.WantedAction.NONE);
  
      // } else if (mControlBoard.getTuck()) {
  
      //     mSuperstructure.setWantTuck(true);
          
      // } else if (mControlBoard.getUntuck()) {
  
      //     mSuperstructure.setWantTuck(false);
  
      // } else if (mControlBoard.getShoot()) {
  
      //     mSuperstructure.setWantShoot();
  
      // } 
      else {
        mIntake.setState(Intake.WantedAction.NONE);
        mHopper.setState(Hopper.WantedAction.NONE);
        mShooter.setVelocity(0);
      }
    } else {

      Climber.WantedAction climber_action = Climber.WantedAction.NONE;

      if (mControlBoard.getClimberJog() == -1){
        climber_action = (Climber.WantedAction.EXTEND);
      } else if(mControlBoard.getClimberJog() == 1){
        climber_action = (Climber.WantedAction.RETRACT);
      }

      mClimber.setState(climber_action);
    }
    

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    mDisabledLooper.stop();
    mEnabledLooper.stop();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
