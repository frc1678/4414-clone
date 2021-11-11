package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
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
        mMaster.set(ControlMode.Velocity, mPeriodicIO.main_velocity / kMainVelocityConversion); //TODO: check between main and upper
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Velocity", mPeriodicIO.main_velocity);
        SmartDashboard.putNumber("Shooter Demand", mPeriodicIO.main_demand);


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
