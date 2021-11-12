package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.lib.util.Util;

import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Turret extends Subsystem {

    private static Turret mInstance;

    private final TalonFX mMaster;
    private final AnalogEncoder mEncoder;
    private final int kEncoderDistancePerRotation = 0; // 25100;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mRunningManual = false;

    protected enum ControlState {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }
    protected ControlState mControlState = ControlState.OPEN_LOOP;

    private static final SupplyCurrentLimitConfiguration CURR_LIM = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);

    private boolean turretWasReset = false;

    private void resetTurretToAbsolute() {
        // if (hoodWasReset) {
        //     return;
        // }
        double absolute_position = getTurretDegreesToTicks(getTicksToTurretDegrees(mEncoder.getDistance()) - (Constants.kTurretEncoderOffset - Constants.kHoodMinLimit));
        mMaster.setSelectedSensorPosition(absolute_position);
        turretWasReset = true;
        System.out.println("resetting hood!");
    }

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }

    private Turret() {
        mMaster = new TalonFX(Constants.kTurretID);
        
        // flywheel motor configs
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false); //TODO: check value
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.setSensorPhase(false);
        
        mMaster.config_kP(0, Constants.kTurretP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kTurretI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kTurretD, Constants.kLongCANTimeoutMs);    
        mMaster.selectProfileSlot(0, 0);

        // flywheel master current limit
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);

        // feedback sensor        
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.2);

        // config encoder
        mEncoder = new AnalogEncoder(new AnalogInput(Constants.kTurretEncoderID));
        mEncoder.reset();
        mEncoder.setDistancePerRotation(kEncoderDistancePerRotation); // ticks per rotation
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Turret.this) {
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    // intermediate methods
    public double getTicksToTurretDegrees(double ticks) {
        return ticks / Constants.kTurretGearRatio;
    }

    public double getTurretDegreesToTicks(double degrees) {
        return degrees * Constants.kTurretGearRatio;
    }

    public synchronized double getTurretEncoderPosition() {
        return getTicksToTurretDegrees(mPeriodicIO.encoder_position);
    }

    public synchronized double getTurretAngle() {
        return getTicksToTurretDegrees(mPeriodicIO.motor_position);
    }

    protected double unitsPerSecondToTicksPer100ms(double units_per_second) {
        return getTurretDegreesToTicks(units_per_second) / 10.0;
    }
    
    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mPeriodicIO.setpoint = getTurretDegreesToTicks(units);
        mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (Constants.kTurretF + Constants.kTurretD / 100.0) / 1023.0;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mMaster.selectProfileSlot(0, 0);
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.encoder_position = mEncoder.getDistance();
        mPeriodicIO.motor_position = mMaster.getSelectedSensorPosition();
        mPeriodicIO.hood_angle = getTurretEncoderPosition();

        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = (int) mMaster.getActiveTrajectoryPosition();

            final int newVel = (int) mMaster.getActiveTrajectoryVelocity();
            if (Util.epsilonEquals(newVel, Constants.kTurretCruiseVelocity, Math.max(1, Constants.kTurretDeadband)) || Util
                    .epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, Constants.kTurretDeadband))) {
                // Mechanism is ~constant velocity.
                mPeriodicIO.active_trajectory_acceleration = 0.0;
            } else {
                // Mechanism is accelerating.
                mPeriodicIO.active_trajectory_acceleration = Math
                        .signum(newVel - mPeriodicIO.active_trajectory_velocity) * Constants.kTurretCruiseAcceleration;
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        }
        
        while (!turretWasReset) {
            resetTurretToAbsolute();
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.setpoint, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.setpoint, DemandType.ArbitraryFeedForward, 0.0);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Encoder Position (ticks)", mPeriodicIO.encoder_position);
        SmartDashboard.putNumber("Turret Encoder Position (degrees)", getTurretEncoderPosition());
        SmartDashboard.putNumber("Turret Motor Position (ticks)", mPeriodicIO.motor_position);
        SmartDashboard.putNumber("Turret Motor Position (degrees)", mPeriodicIO.motor_position / Constants.kTurretGearRatio);
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
    }

    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.setpoint = demand;
        mRunningManual = true;
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double encoder_position;
        public double motor_position;
        public double hood_angle;
        public int active_trajectory_position; // ticks
        public int active_trajectory_velocity; // ticks/100ms
        public double active_trajectory_acceleration; // ticks/100ms/s
        public double feedforward;

        //OUTPUTS
        public double setpoint;
    }

}
