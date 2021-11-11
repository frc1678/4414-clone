package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hopper extends Subsystem {

    private final TalonFX mMaster;
    private final TalonFX mSecondary;
    private final DigitalInput mSensor;

    private static Hopper mInstance;

    private final PeriodicIO mPeriodicIO;

    private State mState = State.IDLE;

    public enum State {
        ELEVATING,
        SHOOTING,
        REVERSING,
        IDLE,
    }

    public enum WantedAction {
        ELEVATE,
        SHOOT,
        REVERSE,
        NONE
    }

    private Hopper() {

        mMaster = new TalonFX(Constants.masterElevatorMotorId);
        mSecondary = new TalonFX(Constants.slaveElevatorMotorId);
        mSensor = new DigitalInput(Constants.elevatorSensorPin);

        mPeriodicIO = new PeriodicIO();

    }

    public static synchronized Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    public void setState(State state) {
        this.mState = state;
    }

    public State getState() {
        return mState;
    }

    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mSecondary.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    }

    public void smartDashboard() {
        SmartDashboard.putString("Hopper state", mState.toString());
        SmartDashboard.putNumber("Hopper demand", mPeriodicIO.demand);
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
                synchronized (Hopper.this) {
                    runStateMachine();
                }
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
            case ELEVATE:
                mState = State.ELEVATING;
                break;
            case SHOOT:
                mState = State.SHOOTING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }

    }

    private void runStateMachine() {
        switch (mState) {
            case ELEVATING:
                // If ball has reached sensor set demand to zero
                if (mSensor.get()) {
                    mPeriodicIO.demand = 0;
                } else {
                    mPeriodicIO.demand = Constants.elevatingDemand;
                }
                break;
            case SHOOTING:
                mPeriodicIO.demand = Constants.hellavatingDemand;
                break;
            case REVERSING:
                mPeriodicIO.demand = -Constants.elevatingDemand;
                break;
            case IDLE:
                mPeriodicIO.demand = 0;
                break;
        }
    }

    public static class PeriodicIO {
        // DEMAND
        public double demand;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hopper State", mState.toString());
        SmartDashboard.putNumber("Hopper Demand", mPeriodicIO.demand);

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
