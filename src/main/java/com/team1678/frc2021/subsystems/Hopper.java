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

    private final PeriodicIO periodicIO;

    private Action state = Action.IDLE;

    private Hopper() {

        mMaster = new TalonFX(Constants.masterElevatorMotorId);
        mSecondary = new TalonFX(Constants.slaveElevatorMotorId);
        mSensor = new DigitalInput(Constants.elevatorSensorPin);

        periodicIO = new PeriodicIO();

    }

    public void setState(WantedAction state) {
        switch (state) {
            case NONE:
                this.state = Action.IDLE;
                break;
            case SHOOT:
                this.state = Action.SHOOTING;
                break;
            case ELEVATE:
                this.state = Action.ELEVATING;
                break;
            case REVERSE:
                this.state = Action.REVERSING;
                break;
        }
    }

    public Action getState() {
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

    public enum Action {
        ELEVATING,
        SHOOTING,
        REVERSING,
        IDLE,
    }

    public enum WantedAction {
        ELEVATE,
        SHOOT,
        REVERSE,
        NONE,
    }

    public static class PeriodicIO {
        // DEMAND
        public double demand;
    }


}
