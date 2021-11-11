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

    private final TalonFX mHopper;
    private final TalonFX mElevator;
    // private final DigitalInput mSensor;

    private static double kHopperVoltage = -7.0;
    private static double kElevatorVoltage = 7.0;

    private static Hopper mInstance;

    private final PeriodicIO mPeriodicIO;

    private State mState = State.IDLE;
    private WantedAction mWantedAction = WantedAction.NONE;

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

        mHopper = new TalonFX(Constants.hopperMotorId);
        mElevator = new TalonFX(Constants.elevatorMotorId);
        // mSensor = new DigitalInput(Constants.elevatorSensorPin);

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

    public WantedAction getWantedAction() {
        return mWantedAction;
    }

    public void writePeriodicOutputs() {
        mHopper.set(ControlMode.PercentOutput, mPeriodicIO.hopperDemand / 12.0);
        mElevator.set(ControlMode.PercentOutput, mPeriodicIO.elevatorDemand / 12.0);
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
        mWantedAction = wanted_state;
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
                // if (mSensor.get()) {
                //     mPeriodicIO.demand = 0;
                // } else {
                //     mPeriodicIO.demand = Constants.elevatingDemand;
                // }
                mPeriodicIO.hopperDemand = kHopperVoltage;
                mPeriodicIO.elevatorDemand = kElevatorVoltage;
                break;
            case SHOOTING:
                mPeriodicIO.hopperDemand = kHopperVoltage;
                mPeriodicIO.elevatorDemand = kElevatorVoltage;
                break;
            case REVERSING:
                mPeriodicIO.hopperDemand = -kHopperVoltage;
                mPeriodicIO.elevatorDemand = -kElevatorVoltage;
                break;
            case IDLE:
                mPeriodicIO.hopperDemand = 0;
                mPeriodicIO.elevatorDemand = 0;
                break;
        }
    }

    public static class PeriodicIO {
        // DEMAND
        public double hopperDemand;
        public double elevatorDemand;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hopper State", mState.toString());
        SmartDashboard.putNumber("Hopper Demand", mPeriodicIO.hopperDemand);
        SmartDashboard.putNumber("Elevator Demand", mPeriodicIO.elevatorDemand);

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
