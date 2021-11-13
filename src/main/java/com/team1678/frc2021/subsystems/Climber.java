package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private static double kExtendingVoltage = -8.0;
    private static double kRetractingVoltage = 8.0;
    private static double kIdleVoltage = 0.0;

    private TalonFX mMaster;

    private static Climber mInstance;

    private Climber() {
        mMaster = new TalonFX(Constants.motorClimberID);
        // mMaster.setNeutralMode(NeutralMode.Brake);
    }


    public enum WantedAction {
        NONE, EXTEND, RETRACT,
    }

    public enum State {
        IDLE, EXTENDING, RETRACTING,
    }

    private State mState = State.IDLE;

    public synchronized void setBrakeMode(boolean brake) {
        mMaster.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.current = mMaster.getStatorCurrent();
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.overrideSoftLimitsEnable(false);
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
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
                synchronized (Climber.this) {
                    runStateMachine();
                }
                final double end = Timer.getFPGATimestamp();
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
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
        }

    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                break;
            case EXTENDING:
                mPeriodicIO.demand = kExtendingVoltage;
                break;
            case RETRACTING:
                mPeriodicIO.demand = kRetractingVoltage;
        }
    }

    private static PeriodicIO mPeriodicIO = new PeriodicIO();


    public static synchronized Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber Voltage", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Climber Current", mPeriodicIO.current);
        SmartDashboard.putString("Climber State", mState.toString());
        SmartDashboard.putNumber("Climber Goal", mPeriodicIO.demand);
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
        public double voltage;
        public double current;

        //OUTPUTS
        public double demand;
    }

    
}