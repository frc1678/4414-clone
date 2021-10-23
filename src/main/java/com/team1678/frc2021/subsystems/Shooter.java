package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

/*import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;*/


public class Shooter implements Subsystem {
    private static Shooter mInstance;

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    private boolean mRunningManual = false;
    private static double kUpperVelocityConversion = 75.0 / 512.0;
    private static double kMainVelocityConversion = 600.0 / 2048.0;

    private PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;

        public double main_velocity;
        public double main_voltage;
        public double main_current;
        public double main_temperature;

        //OUTPUTS
        public double main_demand;
    }

    private Shooter() {
        mMaster = new TalonFX(Constants.kMasterFlywheelID);
        mSlave = new TalonFX(Constants.kSlaveFlywheelID);

        mPeriodicIO = new PeriodicIO();

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0);
        mMaster.enableVoltageCompensation(true);

        mMaster.config_kP(0, Constants.kShooterP);
        mMaster.config_kI(0, Constants.kShooterI);
        mMaster.config_kD(0, Constants.kShooterD);
        mMaster.config_kF(0, Constants.kShooterF);
        mMaster.config_IntegralZone(0, (int) (200.0 / kUpperVelocityConversion));
        mMaster.selectProfileSlot(0, 0);

        mSlave.setInverted(true);
    }

    public synchronized static Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }


    public synchronized double getShooterRPM() {
        return mMaster.getSelectedSensorVelocity();
    }


    public synchronized void setOpenLoopOverhead(double main) {
        mPeriodicIO.main_demand = main;
        mRunningManual = true;
    }

    public synchronized double getDemandOverhead() {
        return mPeriodicIO.main_demand;
    }


    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.main_demand = velocity;
        mRunningManual = false;

    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        //for main wheel
        mPeriodicIO.main_velocity = mMaster.getSelectedSensorVelocity() * kMainVelocityConversion;
        mPeriodicIO.main_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.main_current = mMaster.getStatorCurrent();
        mPeriodicIO.main_temperature = mMaster.getTemperature();
    }


    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.main_demand / kMainVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Main Wheel Velocity", mPeriodicIO.main_velocity);
        SmartDashboard.putNumber("Main Wheel Current", mPeriodicIO.main_current);
        SmartDashboard.putNumber("Main Wheel Goal", mPeriodicIO.main_demand);
        SmartDashboard.putNumber("Main Wheel Temperature", mPeriodicIO.main_temperature);
    }

    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

}

