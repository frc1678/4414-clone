package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends Subsystem {

    private static final double kRetractingVoltage = -2.0;
    private static final double kExtendingVoltage = 2.0;
    private static final double kClimbingVoltage = -4.0;
    private static final double kIdleVoltage = 0.0;
    private static Climber mInstance;
    private double mInitialTime;

    private boolean mExtended = false;

    private PeriodicIO mPeriodicIO;

    private final TalonFX mMaster;

    public enum WantedAction {
        NONE , RETRACT, EXTEND, CLIMB,
    }

    private static WantedAction mWantedAction;

    public enum State {
        IDLE , RETRACTING, EXTENDING, CLIMBING,
    }

    private State mState = State.IDLE;

    private Climber() {
        mMaster = new TalonFX(Constants.motorClimberID);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeouts);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeouts);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeouts);
        mMaster.configMotionCruiseVelocity(30000, Constants.kLongCANTimeouts);
        mMaster.config_kP(0, 0.5);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeouts);

    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public synchronized void setBrakeMode(boolean brake) {
        mMaster.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                break;
            case RETRACTING:
                mPeriodicIO.demand = kRetractingVoltage;
                break;
            case EXTENDING:
                mPeriodicIO.demand = kExtendingVoltage; {
                    mExtended = true;
                }
                break;
            case CLIMBING:
                mPeriodicIO.demand = kClimbingVoltage;
                break;
        }
    }

    public void setState(WantedAction wanted_state) {
        mWantedAction = wanted_state;

        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case CLIMB:
                mState = State.CLIMBING;
                break;
        }
    }

    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    }

    public static class PeriodicIO {
        // INPUTS
        public double position;
        public double velocity;
        public double current;
        public double voltage;

        // OUTPUTS
        public double demand;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putNumber("ClimbOutputVoltage", mMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Climber Voltage", mPeriodicIO.voltage);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Climber Current", mPeriodicIO.current);
        SmartDashboard.putString("Climber Wanted Action", mWantedAction.name());
        SmartDashboard.putNumber("Climber Goal", mPeriodicIO.demand);

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
