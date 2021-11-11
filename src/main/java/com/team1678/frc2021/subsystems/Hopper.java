package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Hopper implements Subsystem {

    private final TalonFX mMaster;
    private final TalonFX mSecondary;
    private final DigitalInput mSensor;

    private static Hopper mInstance;

    private final PeriodicIO periodicIO;

    private State state = State.IDLE;

    private Hopper() {

        mMaster = new TalonFX(Constants.masterElevatorMotorId);
        mSecondary = new TalonFX(Constants.slaveElevatorMotorId);
        mSensor = new DigitalInput(Constants.elevatorSensorPin);

        periodicIO = new PeriodicIO();

    }

    public static synchronized Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        runStateMachine();
        writePeriodicOutputs();
        smartDashboard();
    }

    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, periodicIO.demand / 12.0);
        mSecondary.set(ControlMode.PercentOutput, periodicIO.demand / 12.0);
    }

    public void smartDashboard() {
        SmartDashboard.putString("Hopper state", state.toString());
        SmartDashboard.putNumber("Hopper demand", periodicIO.demand);
    }

    private void runStateMachine() {
        switch (state) {
            case ELEVATING:
                // If ball has reached sensor set demand to zero
                if (mSensor.get()) {
                    periodicIO.demand = 0;
                } else {
                    periodicIO.demand = Constants.elevatingDemand;
                }
                break;
            case SHOOTING:
                periodicIO.demand = Constants.hellavatingDemand;
                break;
            case REVERSING:
                periodicIO.demand = -Constants.elevatingDemand;
                break;
            case IDLE:
                periodicIO.demand = 0;
                break;
        }
    }

    public enum State {
        ELEVATING,
        SHOOTING,
        REVERSING,
        IDLE,
    }

    public static class PeriodicIO {
        // DEMAND
        public double demand;
    }


}
