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

    private final TalonFX mFlywheel;
    private final TalonFX mOverhead;

    private boolean mRunningManual = false;

    private static double kFlywheelVelocityConversion = 150.0 / 2048.0;
    private static double kOverheadVelocityConversion = 150.0 / 2048.0;

    private static double kFlywheelTolerance = 200.0;
    private static double kOverheadTolerance = 200.0;

    private PeriodicIO mPeriodicIO;

    private Shooter() {
        mFlywheel = new TalonFX(Constants.kMasterFlywheelID);
        mOverhead = new TalonFX(Constants.kOverheadFlywheelID);

        mPeriodicIO = new PeriodicIO();

        // flywheel motor configs
        mFlywheel.set(ControlMode.PercentOutput, 0);
        mFlywheel.setInverted(true); //TODO: check value
        mFlywheel.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mFlywheel.enableVoltageCompensation(true);
        
        mFlywheel.config_kP(0, Constants.kShooterFlywheelP, Constants.kLongCANTimeoutMs);
        mFlywheel.config_kI(0, Constants.kShooterFlywheelI, Constants.kLongCANTimeoutMs);
        mFlywheel.config_kD(0, Constants.kShooterFlywheelD, Constants.kLongCANTimeoutMs);
        mFlywheel.config_kF(0, Constants.kShooterFlywheelF, Constants.kLongCANTimeoutMs);
        mFlywheel.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        mFlywheel.selectProfileSlot(0, 0);

        // flywheel master current limit
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mFlywheel.configSupplyCurrentLimit(curr_lim);

        // overhead motor configs
        mOverhead.set(ControlMode.PercentOutput, 0);
        mOverhead.setInverted(false); //TODO: check value
        mOverhead.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mOverhead.enableVoltageCompensation(true);
        
        mOverhead.config_kP(0, Constants.kShooterOverheadP, Constants.kLongCANTimeoutMs);
        mOverhead.config_kI(0, Constants.kShooterOverheadI, Constants.kLongCANTimeoutMs);
        mOverhead.config_kD(0, Constants.kShooterOverheadD, Constants.kLongCANTimeoutMs);
        mOverhead.config_kF(0, Constants.kShooterOverheadF, Constants.kLongCANTimeoutMs);
        mOverhead.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        mOverhead.selectProfileSlot(0, 0);
 
        // feedback sensor        
        mFlywheel.set(ControlMode.PercentOutput, 0);
        mFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFlywheel.configClosedloopRamp(0.2);
        mOverhead.set(ControlMode.PercentOutput, 0);
        mOverhead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mOverhead.configClosedloopRamp(0.2);
    }

    public synchronized static Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }


    public synchronized double getShooterRPM() {
        return mFlywheel.getSelectedSensorVelocity();
    }


    // /* set open loop demand */
    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.flywheel_demand = demand;
        mPeriodicIO.overhead_demand = -demand;
        mRunningManual = true;
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.flywheel_demand = velocity;
        mPeriodicIO.overhead_demand = -velocity;
        mRunningManual = false;
    }

    public synchronized boolean spunUp() {
        if (Math.abs(mPeriodicIO.flywheel_demand) > 0 && Math.abs(mPeriodicIO.overhead_demand) > 0) {
            return Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.flywheel_velocity, kFlywheelTolerance) &&
                    Util.epsilonEquals(mPeriodicIO.overhead_demand, mPeriodicIO.overhead_velocity, kOverheadTolerance);
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
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mFlywheel.set(ControlMode.Velocity, mPeriodicIO.flywheel_demand / kFlywheelVelocityConversion);
            mOverhead.set(ControlMode.Velocity, mPeriodicIO.overhead_demand / kOverheadVelocityConversion);
        } else {
            mFlywheel.set(ControlMode.PercentOutput, 0);
            mOverhead.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Flywheel Velocity", mPeriodicIO.flywheel_velocity);
        SmartDashboard.putNumber("Flywheel Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Flywheel Current", mPeriodicIO.flywheel_current);
        SmartDashboard.putNumber("Flywheel Goal", mPeriodicIO.flywheel_demand);
        SmartDashboard.putNumber("Flywheel Temperature", mPeriodicIO.flywheel_temperature);

        SmartDashboard.putNumber("Overhead Velocity", mPeriodicIO.overhead_velocity);
        SmartDashboard.putNumber("Overhead Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Overhead Current", mPeriodicIO.overhead_current);
        SmartDashboard.putNumber("Overhead Goal", mPeriodicIO.overhead_demand);
        SmartDashboard.putNumber("Overhead Temperature", mPeriodicIO.overhead_temperature);

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
        public double timestamp;
        
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
        public double flywheel_temperature;

        public double overhead_velocity;
        public double overhead_voltage;
        public double overhead_current;
        public double overhead_temperature;

        //OUTPUTS
        public double flywheel_demand;
        public double overhead_demand;
    }
    
}
