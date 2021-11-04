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
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem {

    private static final double kRetractingVoltage = 200.0;
    private static final double kExtendingVoltage = 2.0;
    private static final double kClimbingVoltage = -2.0;
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
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(30000, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(0, 0.5);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);

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

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        mState = State.IDLE;
    }

    @Override
    public void simulationPeriodic() {
        synchronized (Climber.this) {
            runStateMachine();
        }
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putNumber("ClimbOutputVoltage", mMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Climber Voltage", mPeriodicIO.voltage);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Climber Current", mPeriodicIO.current);
        SmartDashboard.putString("Climber Wanted Action", mWantedAction.name());
        SmartDashboard.putNumber("Climber Goal", mPeriodicIO.demand);
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
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
                }
        }

    public void setState(WantedAction wanted_state) {
        mWantedAction = wanted_state;

        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
            case RETRACT:
                mState = State.RETRACTING;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case CLIMB:
                mState = State.CLIMBING;
                break;
        }
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

}

