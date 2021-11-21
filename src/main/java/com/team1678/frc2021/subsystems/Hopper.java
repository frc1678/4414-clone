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

    private static double kHopperVoltage = -7.0;
    private static double kElevatorVoltage = 7.0;

    private static Hopper mInstance;
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private State mState = State.IDLE;
    private WantedAction mWantedAction = WantedAction.NONE;

    public enum WantedAction {
        // INDEX,
        FEED,
        REVERSE,
        NONE
    }
    
    public enum State {
        // INDEXING,
        FEEDING,
        REVERSING,
        IDLE,
    }

    private Hopper() {
        mHopper = new TalonFX(Constants.hopperMotorId);
        mElevator = new TalonFX(Constants.elevatorMotorId);
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

    @Override
    public synchronized void readPeriodicInputs() {
    }

    @Override
    public void writePeriodicOutputs() {
        mHopper.set(ControlMode.PercentOutput, mPeriodicIO.hopper_demand / 12.0);
        mElevator.set(ControlMode.PercentOutput, mPeriodicIO.elevator_demand / 12.0);
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
            /*
            case INDEX:
                mState = State.INDEXING;
                break;
            */
            case FEED:
                mState = State.FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }

    }

    private void runStateMachine() {
        switch (mState) {
            /*
            case INDEXING:
                mPeriodicIO.hopper_demand = kHopperVoltage;
                mPeriodicIO.elevator_demand = kElevatorVoltage;                
                break;
            */
            case FEEDING:
                mPeriodicIO.hopper_demand = kHopperVoltage;
                mPeriodicIO.elevator_demand = kElevatorVoltage;
                break;
            case REVERSING:
                mPeriodicIO.hopper_demand = -kHopperVoltage;
                mPeriodicIO.elevator_demand = -kElevatorVoltage;
                break;
            case IDLE:
                mPeriodicIO.hopper_demand = 0;
                mPeriodicIO.elevator_demand = 0;
                break;
        }
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

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hopper State", mState.toString());
        SmartDashboard.putNumber("Hopper Demand", mPeriodicIO.hopper_demand);
        SmartDashboard.putNumber("Elevator Demand", mPeriodicIO.elevator_demand);

        SmartDashboard.putNumber("Floor Voltage", mPeriodicIO.floor_voltage);
        SmartDashboard.putNumber("Floor Current", mPeriodicIO.floor_current);
        SmartDashboard.putNumber("Elevator Voltage", mPeriodicIO.elevator_voltage);
        SmartDashboard.putNumber("Elevator Current", mPeriodicIO.elevator_current);
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double floor_voltage;
        public double floor_current;
        public double elevator_voltage;
        public double elevator_current;

        // OUTPUTS
        public double hopper_demand;
        public double elevator_demand;
    }
}
