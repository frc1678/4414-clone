package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    private static double kIntakingVoltage = 9.0;
    private static double kReversingVoltage = -9.0;
    private static double kIdleVoltage = 0.0;
    private TalonFX mMaster = new TalonFX(Constants.masterIntakeMotorId);
    private static double mCurrent;

    private static Intake mInstance;


    public enum WantedAction {
        NONE, INTAKE, REVERSE,
    }

    public enum State {
        IDLE, INTAKING, REVERSING,
    }

    private State mState = State.IDLE;


    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);

    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public double getCurrent() {
        return mCurrent;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();
                synchronized (Intake.this) {
                    runStateMachine();
                }
                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INTAKE:
                mState = State.INTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }

    }

    public void runStateMachine() {
        switch (mState) {
            case INTAKING:
                if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = kIntakingVoltage;
                } else {
                    mPeriodicIO.demand = 0.0;
                }
                mPeriodicIO.deploy = true;
                break;
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.deploy = false;
                break;
            case REVERSING:
                if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = kReversingVoltage;
                } else {
                    mPeriodicIO.demand = 0.0;
                }
        }
    }

    private static PeriodicIO mPeriodicIO = new PeriodicIO();


    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putNumber("Intake Voltage", getVoltage());
        SmartDashboard.putString("Intake State", mState.toString());
        SmartDashboard.putBoolean("Solenoid Goal", mPeriodicIO.deploy);

    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    public static class PeriodicIO {
        //INPUTS
        public double current;
        public boolean intake_out;
        public double dt;

        //OUTPUTS
        public double demand;
        public boolean deploy;
    }

    
}