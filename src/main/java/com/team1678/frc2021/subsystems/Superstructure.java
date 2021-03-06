package com.team1678.frc2021.subsystems;

import java.util.OptionalDouble;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.ControlBoard;
import com.team254.lib.util.Util;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem{

    // Superstructure instance
    private static Superstructure mInstance;

    private Superstructure () {
        // empty
    }

    public static synchronized Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    /* Required subsystem instances */
    private final Intake mIntake = Intake.getInstance();
    private final Hopper mHopper = Hopper.getInstance();
    // private final Hood mHood = Hood.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    /* Setpoint variables */
    private double mHoodSetpoint = 0.0; //TODO: check value
    private double mTurretSetpoint = 0.0;
    private double mTurretHint = 0.0;
    private double mShooterSetpoint = 1000.0;
    private double mHoodAngleAdjustment = 0.0;

    // Status variables for functions
    private boolean mIsSpunUp = false;
    private double current_turret = 0.0;
    private double target_offset = 0.0;
    private double formal_turret = 0.0;
    private double formal_shooter = 0.0;
    private double formal_hood = 0.0;
    private Hopper.WantedAction formal_hopper = Hopper.WantedAction.NONE;

    // Superstructure constants
    private final double kSpitVelocity = 200; //TODO: check value

    /* SUPERSTRUCTURE FUNCTIONS */
    private boolean mWantsTuck = false;
    private boolean mWantsPrep = false;
    private boolean mWantsShoot = false;
    private boolean mWantsSpit = false;
    private boolean mWantsVisionAim = false;
    private boolean mWantsTurretHint = false;
    private boolean mResetHoodAngleAdjustment = false;

    // Function setters
    public synchronized void setWantTuck(boolean tuck) {
        mWantsTuck = tuck;
        mWantsPrep = false;
        mWantsShoot = false;
    }

    public synchronized void setWantPrep() {
        mWantsPrep = !mWantsPrep;
        mWantsShoot = false;
    }

    public synchronized void setWantShoot() {
        mWantsPrep = false;
        mWantsShoot = !mWantsShoot;
    }

    public synchronized void setWantShoot(boolean shoot) {
        mWantsShoot = shoot;
    }

    public synchronized void setWantTestSpit() {
        mWantsSpit = !mWantsSpit;
    }

    public synchronized void setmWantVisionAim(boolean vision_aim) {
        mWantsVisionAim = vision_aim;
    }

    public synchronized void setWantTurretHint(boolean wants_hint, double hint) {
        mWantsTurretHint = wants_hint;
        mTurretHint = hint;
    }

    public boolean getWantsVisionAim() {
        return mWantsVisionAim;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // TODO: Add anything necessary
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    maybeUpdateGoalFromVision(timestamp);
                    setSetpoints();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop(); // TODO: Add anything necessary
            }
        });
    }

    /* UPDATE SHOOTER AND HOOD GOAL WHEN VISION AIMING */
    public synchronized void maybeUpdateGoalFromVision(double timestamp) {
        if (mWantsTurretHint) {
            mTurretSetpoint = mTurretHint;
            mWantsTurretHint = false;
        } 
        
        if (mLimelight.seesTarget()) {

            double currentAngle = mTurret.getAngle();
            double targetOffset = mLimelight.getTargetOffset().getAsDouble();
            current_turret = currentAngle;
            target_offset = targetOffset;

            if (mWantsVisionAim) {
                mTurretSetpoint = currentAngle - targetOffset;
            } else {
                mTurretSetpoint = 0.0;
            }
        }
    }

    /* UPDATE AND SET ALL SETPOINTS */
    public void setSetpoints() {
        /* Default Hopper wanted action to be set */
        Hopper.WantedAction real_hopper;
        /* Real hood angle setpoint to be set */
        double real_hood;
        double real_turret;
        /* Real shooter velocity setpoint to be set */
        double real_shooter;
        // status variable tracker for whether shooter is spun up
        mIsSpunUp = mShooter.spunUp(); 

        if (mWantsTuck) {
            real_hopper = Hopper.WantedAction.NONE;
            real_hood = Constants.kHoodMinLimit;
            real_shooter = 0.0;
            real_turret = 0.0;
        } else if (mWantsPrep) {
            real_hopper = Hopper.WantedAction.NONE;
            real_hood = mHoodSetpoint;
            real_shooter = mShooterSetpoint;
            real_turret = mTurretSetpoint;
        } else if (mWantsShoot) {
            real_hood = mHoodSetpoint;
            real_shooter = mShooterSetpoint;
            real_hopper = Hopper.WantedAction.FEED;
            real_turret = mTurretSetpoint;
        } else if (mWantsSpit) {
            real_hood = Constants.kHoodMinLimit;
            real_shooter = kSpitVelocity;
            real_hopper = Hopper.WantedAction.FEED;
            real_turret = 0.0;
        } else {
            real_hopper = Hopper.WantedAction.NONE;

            real_hood = mHoodSetpoint;
            real_turret = mTurretSetpoint;

            real_shooter = 0;
        }
        
        /* FOLLOW TURRET SETPOINT GOAL */
        mTurret.setSetpointMotionMagic(real_turret);

        /* FOLLOW HOOD SETPOINT GOAL */
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(mShooterSetpoint);
        }

        /* SET Hopper STATE */
        if (mIntake.getState() == Intake.State.REVERSING) {
            mHopper.setState(Hopper.WantedAction.REVERSE);
        } else if (mShooter.spunUp()) {
            mHopper.setState(real_hopper);
        } else {
            mHopper.setState(Hopper.WantedAction.NONE);
        }

        // update tracker variables for goals for smart dashboard readings
        formal_hood = real_hood;
        formal_shooter = real_shooter;
        formal_hopper = real_hopper;
        formal_turret = real_turret;
    }   

    @Override
    public void outputTelemetry() {
        // Formal goals for hood, shooter, and Hopper that are followed
        SmartDashboard.putNumber("Hood Goal", formal_hood);
        SmartDashboard.putNumber("Hood Setpoint", mHoodSetpoint);
        SmartDashboard.putNumber("Turret Goal", formal_turret);
        SmartDashboard.putNumber("Turret Setpoint", mTurretSetpoint);
        SmartDashboard.putNumber("Shooter Goal", formal_shooter);
        SmartDashboard.putNumber("Shooter Setpoint", mShooterSetpoint);
        SmartDashboard.putString("Hopper Goal", formal_hopper.toString());

        // Other status tracker variables
        SmartDashboard.putBoolean("Is Spun Up", mIsSpunUp);
        SmartDashboard.putNumber("Current Turret Angle", current_turret);
        SmartDashboard.putNumber("Target Offset", target_offset);
        
        // Formal superstructure function values
        SmartDashboard.putBoolean("Wants Tuck", mWantsTuck);
        SmartDashboard.putBoolean("Wants Prep", mWantsPrep);
        SmartDashboard.putBoolean("Wants Shoot", mWantsShoot);
        SmartDashboard.putBoolean("Wants Spit", mWantsSpit);
        SmartDashboard.putBoolean("Wants Vision Aim", mWantsVisionAim);
        SmartDashboard.putBoolean("Limelight Sees Target", mLimelight.seesTarget());
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }  

}
