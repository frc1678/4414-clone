package com.team1678.frc2021;

import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.frc2021.subsystems.Hood;
import com.team1678.frc2021.subsystems.Hopper;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Subsystem;
import com.team1678.frc2021.subsystems.Turret;
import com.team1678.frc2021.subsystems.Hopper.WantedAction;

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
    private final Hood mHood = Hood.getInstance();
    private final Hopper mHopper = Hopper.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Turret mTurret = Turret.getInstance();

    /* Setpoint variables */
    private double mHoodSetpoint = 50.0; //TODO: check value
    private double mShooterSetpoint = 0.0; 

    // Superstructure constants
    private final double kSpitVelocity = 200; //TODO: check value

    /* SUPERSTRUCTURE FUNCTIONS */
    private boolean mWantsTuck = false;
    private boolean mWantsScan = false;
    private boolean mWantsPrep = false;
    private boolean mWantsShoot = false;
    private boolean mWantsSpit = false;

    // Status variables for functions
    private boolean mIsSpunUp = false;

    // Function setters
    public synchronized void setWantTuck(boolean tuck) {
        mWantsTuck = tuck;
        mWantsPrep = false;
        mWantsShoot = false;
        mWantsScan = false;
    }

    public synchronized void setWantHoodScan(boolean scan) {
        if (scan != mWantsScan) {
            if (scan) {
                mHoodSetpoint = Constants.kHoodMinLimit + 10;
            } else {
                mHoodSetpoint = mHood.getAngle();
            }
        }
        mWantsScan = scan;
    }

    public synchronized void setWantPrep() {
        mWantsPrep = !mWantsPrep;
        mWantsShoot = false;
    }

    public synchronized void setWantShoot() {
        mWantsPrep = false;
        mWantsShoot = !mWantsShoot;
    }

    public synchronized void setWantTestSpit() {
        mWantsSpit = !mWantsSpit;
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
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop(); // TODO: Add anything necessary
            }
        });
    }

    public void setSetpoints() {
        double hood_value = mHood.getPosition();
        double shooter_value = mShooter.getShooterRPM();

        Hopper.WantedAction hopper_state = mHopper.getWantedAction();

        if (mWantsTuck) {
            hood_value = Constants.kHoodMinLimit;
            shooter_value = 0.0;
        } else if (mWantsPrep) {

        } else if (mWantsShoot) {
            hood_value = mHoodSetpoint;
            shooter_value = mShooterSetpoint;
            hopper_state = WantedAction.SHOOT;

        } else if (mWantsSpit) {

        }

        /* FOLLOW SETPOINT GOALS*/
        mHood.setPosition(hood_value);
        mShooter.setVelocity(shooter_value);
        mHopper.setState(hopper_state);
    }

    @Override
    public void outputTelemetry() {
        // Other status tracker variables
        SmartDashboard.putBoolean("Is Spun Up", mIsSpunUp);
        
        // Formal superstructure function values
        SmartDashboard.putBoolean("Wants Tuck", mWantsTuck);
        SmartDashboard.putBoolean("Wants Scan", mWantsScan);
        SmartDashboard.putBoolean("Wants Prep", mWantsPrep);
        SmartDashboard.putBoolean("Wants Shoot", mWantsShoot);
        SmartDashboard.putBoolean("Wants Spit", mWantsSpit);

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
