package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.Timer;

public class Shooter implements Subsystem {
    private static Shooter mInstance;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final TalonFX mMasterOverhead;

    private boolean mRunningManual = false;
    private static double kUpperVelocityConversion = 75.0 / 512.0;
    private static double kMainVelocityConversion = 600.0 / 2048.0;
    private static double kShooterTolerance = 200.0;

    private PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;

        public double upper_velocity;
        public double upper_voltage;
        public double upper_current;
        public double upper_temperature;
        public double main_velocity;
        public double main_voltage;
        public double main_current;
        public double main_temperature;

        //OUTPUTS
        public double upper_demand;
        public double main_demand;
    }

    private Shooter() {
        //Flywheel
        mMaster = new TalonFX(Constants.kMasterFlywheelID);
        mSlave = new TalonFX(Constants.kSlaveFlywheelID);
        mMasterOverhead = new TalonFX(Constants.kOverheadFlywheelID);

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


        mSlave.setInverted(false);

        //for overhead motor
        mMasterOverhead.set(ControlMode.PercentOutput, 1);
        mMasterOverhead.setInverted(false);
        mMasterOverhead.configVoltageCompSaturation(12.0);
        mMasterOverhead.enableVoltageCompensation(true);

        mMasterOverhead.config_kP(1, Constants.kShooterP);
        mMasterOverhead.config_kI(1, Constants.kShooterI);
        mMasterOverhead.config_kD(1, Constants.kShooterD);
        mMasterOverhead.config_kF(1, Constants.kShooterF);
        mMasterOverhead.config_IntegralZone(1, (int) (200.0 / kMainVelocityConversion));
        mMasterOverhead.selectProfileSlot(1, 0);

        mMasterOverhead.set(ControlMode.PercentOutput, 0);
        mMaster.configClosedloopRamp(0.2);
        mMasterOverhead.configClosedloopRamp(0.2);
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.upper_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.upper_demand, mPeriodicIO.upper_velocity, kShooterTolerance);
        }
        return false;
    }
    public synchronized static Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public synchronized void setOpenLoop(double upper) {
        mPeriodicIO.upper_demand = upper;
        mRunningManual = true;
    }

    public synchronized double getDemand() {
        return mPeriodicIO.upper_demand;
    }

    public synchronized double getShooterRPM() {
        return mMaster.getSelectedSensorVelocity();
    }

    public synchronized double getVelocity() {
        return mPeriodicIO.upper_velocity;
    }

    //for main wheel
    public synchronized void setVelocityOverhead(double velocity) {
        mPeriodicIO.main_demand = velocity;
        mRunningManual = false;
    }

    public synchronized void setOpenLoopOverhead(double main) {
        mPeriodicIO.main_demand = main;
        mRunningManual = true;
    }

    public synchronized double getDemandOverhead() {
        return mPeriodicIO.main_demand;
    }

    public synchronized double getShooterRPMOverhead() {
        return mMasterOverhead.getSelectedSensorVelocity();
    }

    public synchronized boolean spunUpOverhead() {
        if (mPeriodicIO.main_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.main_demand, mPeriodicIO.main_velocity, kShooterTolerance);
        }
        return false;
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.upper_demand = velocity;
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

        //for upper wheel
        mPeriodicIO.upper_velocity = mMaster.getSelectedSensorVelocity() * kUpperVelocityConversion;
        mPeriodicIO.upper_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.upper_current = mMaster.getStatorCurrent();
        mPeriodicIO.upper_temperature = mMaster.getTemperature();

        //for main wheel
        mPeriodicIO.main_velocity = mMasterOverhead.getSelectedSensorVelocity() * kMainVelocityConversion;
        mPeriodicIO.main_voltage = mMasterOverhead.getMotorOutputVoltage();
        mPeriodicIO.main_current = mMasterOverhead.getStatorCurrent();
        mPeriodicIO.main_temperature = mMasterOverhead.getTemperature();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {

        //for upper wheel
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.upper_demand / kUpperVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
        }

        //for main wheel
        if (!mRunningManual) {
            mMasterOverhead.set(ControlMode.Velocity, mPeriodicIO.main_demand / kMainVelocityConversion);
        } else {
            mMasterOverhead.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public synchronized void outputTelemetry() {

        //for upper wheel
        SmartDashboard.putNumber("Upper Wheel Velocity", mPeriodicIO.upper_velocity);
        SmartDashboard.putNumber("Upper Wheel Current", mPeriodicIO.upper_current);
        SmartDashboard.putNumber("Upper Wheel Goal", mPeriodicIO.upper_demand);
        SmartDashboard.putNumber("Upper Wheel Temperature", mPeriodicIO.upper_temperature);

        //for main wheel
        SmartDashboard.putNumber("Main Wheel Velocity", mPeriodicIO.main_velocity);
        SmartDashboard.putNumber("Main Wheel Current", mPeriodicIO.main_current);
        SmartDashboard.putNumber("Main Wheel Goal", mPeriodicIO.main_demand);
        SmartDashboard.putNumber("Main Wheel Temperature", mPeriodicIO.main_temperature);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

}
