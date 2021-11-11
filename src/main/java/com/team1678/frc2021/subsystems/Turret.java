package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import javax.annotation.OverridingMethodsMustInvokeSuper;

public class Turret implements Subsystem {
    private static Turret mInstance;
    private Encoder mturretEncoder;
    private double mOffset = 0;
    private boolean mHoming = true;
    public static final boolean kUseManualHomingRoutine = false;
    private boolean ismHoming = true;

    private TalonFX mMaster = new TalonFX(Constants.turretMotorId);
    private DigitalInput mLimitSwitch = new DigitalInput(1);
    private final Encoder getMturretEncoder = new Encoder(0,1);

    private static final SupplyCurrentLimitConfiguration CURR_LIM = new SupplyCurrentLimitConfiguration(true, 40,60,0.01);

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }

    double Turret() {
        //there's only one TalonFX, so we can call just call it "Turret"
        mMaster.set(ControlMode.PercentOutput, 0.62);

        //sets the software limits for the spin of the turret
        double constrainTicks;
        double ticks; {
            double mReverseSoftLimitTicks = (Constants.MinRadAngle / (235.0 * Constants.RotationsPerTick));
            double mForwardSoftLimitTicks = Constants.MaxRadAngle / (235.0 * Constants.RotationsPerTick);
            return Util.cap(mReverseSoftLimitTicks, mForwardSoftLimitTicks);
        }
        
    }

    public synchronized boolean ismHoming() {
        return mHoming;
    }

    private void updateHoming() {
        mHoming =! new Turret() {
            @Override
            public synchronized boolean ismHoming() {
                return super.ismHoming();
            }

            public boolean checkSystem() {
                return false;
            }
        }.checkSystem();
    }

    public synchronized void autonomousPeriodic() {
        if(mHoming) {
            System.out.println("is calibrated");
            mturretEncoder.reset();

            //Motor to encoder

            mMaster.setSelectedSensorPosition(mturretEncoder.getDistance());
            mMaster.setSelectedSensorPosition((int) getAngle());

            mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHoming = false;
        }else {
            mturretEncoder.reset();
        }
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized double getPosition(){
        return mturretEncoder.getRaw();
    }

    private boolean atHomingLocation() {
        return mHoming;
    }

    @Override
    public void periodic() {
        // Runs every tick
    }
}