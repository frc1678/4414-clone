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
import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Turret;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
  private final Superstructure mSuperstructure = Superstructure.getInstance();
  private final Hood mHood = Hood.getInstance();
  private final Hopper mHopper = Hopper.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Turret mTurret = Turret.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  // private final Swerve mSwerve = Swerve.getInstance();

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
      mSuperstructure,
      mIntake,
      mHopper,
      mTurret,
      mHood,
      mShooter,
      mClimber,
      mLimelight
    ); 

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mSubsystemManager.registerDisabledLoops(mDisabledLooper);


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
    
    // CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Controller Rotation",m_robotContainer.getRotationAxis());
    SmartDashboard.putBoolean("Intake Command", mControlBoard.getRunIntake());
    SmartDashboard.putBoolean("Climb Mode", climbMode);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mEnabledLooper.stop();
    mDisabledLooper.start();
    mLimelight.setLed(Limelight.LedMode.ON);
  }

  @Override
  public void disabledPeriodic() {
    mLimelight.setLed(Limelight.LedMode.OFF);
    m_robotContainer.resetAngleToAbsolute();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    mLimelight.setLed(Limelight.LedMode.ON);
    mLimelight.setPipeline(Constants.kPortPipeline);

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

    mLimelight.setLed(Limelight.LedMode.ON);
    mLimelight.setPipeline(Constants.kPortPipeline);
            

    mClimber.setBrakeMode(true);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (mControlBoard.getClimbMode()) {
      climbMode = true;
    }

    mSuperstructure.setmWantVisionAim(mControlBoard.getVisionAim());

    if (!climbMode) {
      if (mControlBoard.getTuck()) {
          mSuperstructure.setWantTuck(true);
      } else if (mControlBoard.getPreShot()) {
          mSuperstructure.setWantPrep();
      } else if (mControlBoard.getShoot()) {
          mIntake.setState(Intake.WantedAction.INTAKE);
          mSuperstructure.setWantShoot();
      } else if (mControlBoard.getTestSpit()) {
          mSuperstructure.setWantTestSpit();
      } else if (mControlBoard.getRunIntake()) {
          mIntake.setState(Intake.WantedAction.INTAKE);
      } else if (mControlBoard.getReverseIntake()) {
          mIntake.setState(Intake.WantedAction.REVERSE);
      } else {
          mIntake.setState(Intake.WantedAction.NONE);
      }
    } else {
      mIntake.setState(Intake.WantedAction.NONE);

      Climber.WantedAction climber_action = Climber.WantedAction.NONE;

      if (mControlBoard.getClimberJog() == -1){
          climber_action = (Climber.WantedAction.EXTEND);
      } else if (mControlBoard.getClimberJog() == 1){
          climber_action = (Climber.WantedAction.RETRACT);
      } else if (mControlBoard.getLeaveClimbMode()) {
          climbMode = false;
      }

      mClimber.setState(climber_action);
    }
    

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    mDisabledLooper.stop();
    mEnabledLooper.stop();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
