package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.lang.invoke.ConstantBootstraps;
import java.util.ArrayList;

public class Intake implements Subsystem {

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

    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);

    }

    public void smartDashboard() {
        runStateMachine();
        writePeriodicOutputs();
        smartDashboard();
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

    public static class PeriodicIO {
        //INPUTS
        public double current;
        public boolean intake_out;
        //OUTPUTS
        public double demand;
        public boolean deploy;
    }

    private static PeriodicIO mPeriodicIO = new PeriodicIO();


    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        synchronized (Intake.this) {
            runStateMachine();
        }
    }

    @Override
    public Command getCurrentCommand() {
        mCurrent = mPeriodicIO.current;
        return null;
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        mState = State.IDLE;
    }

    @Override
    public Command getDefaultCommand() {
        return CommandScheduler.getInstance().requiring(this);
    }
}