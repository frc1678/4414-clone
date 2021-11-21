package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.lib.util.Util;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private final TalonFX mMaster;
    private TalonFX mSlave;

    private boolean mRunningManual = false;

    private static double kFlywheelVelocityConversion = 150.0 / 2048.0;
    private static double kOverheadVelocityConversion = 150.0 / 2048.0;

    private static double kFlywheelTolerance = 200.0;
    private static double kOverheadTolerance = 200.0;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private Shooter() {
        mMaster = new TalonFX(Constants.kMasterFlywheelID);
        mSlave = new TalonFX(Constants.kOverheadFlywheelID);
        mSlave.follow(mMaster);

        // flywheel motor configs
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true); //TODO: check value
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        
        mMaster.config_kP(0, Constants.kShooterFlywheelP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kShooterFlywheelI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kShooterFlywheelD, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kShooterFlywheelF, Constants.kLongCANTimeoutMs);
        mMaster.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        mMaster.selectProfileSlot(0, 0);

        // flywheel master current limit
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);

        // feedback sensor        
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.2);
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


    // /* set open loop demand */
    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.flywheel_demand = demand;
        mRunningManual = true;
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.flywheel_demand = velocity;
        mRunningManual = false;
    }

    public synchronized boolean spunUp() {
        if (Math.abs(mPeriodicIO.flywheel_demand) > 0) {
            return Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.flywheel_velocity, kFlywheelTolerance);
            }
        return false;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void readPeriodicInputs() {        
        mPeriodicIO.flywheel_velocity = mMaster.getSelectedSensorVelocity() * kFlywheelVelocityConversion;
        mPeriodicIO.flywheel_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.flywheel_current = mMaster.getSupplyCurrent();
        mPeriodicIO.flywheel_temperature = mMaster.getTemperature();

    }

    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.flywheel_demand / kFlywheelVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Flywheel Velocity", mPeriodicIO.flywheel_velocity);
        SmartDashboard.putNumber("Flywheel Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Flywheel Current", mPeriodicIO.flywheel_current);
        SmartDashboard.putNumber("Flywheel Goal", mPeriodicIO.flywheel_demand);
        SmartDashboard.putNumber("Flywheel Temperature", mPeriodicIO.flywheel_temperature);

        SmartDashboard.putBoolean("Shooter Spun Up: ", spunUp());
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

    public static class PeriodicIO {
        //INPUTS
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
        public double flywheel_temperature;

        //OUTPUTS
        public double flywheel_demand;
    }
    
}
