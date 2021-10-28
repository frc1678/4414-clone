package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.Util;
import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.lib.util.HallCalibration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Turret implements Subsystem {
    private static Turret mInstance;
    private Encoder turretEncoder;
    private double mOffset = 0;
    private boolean mHoming = true;
    public static final boolean kUseManualHomingRoutine = false;

    private TalonFX mMaster = new TalonFX(Constants.turretMotorId);
    private HallCalibration calibration = new HallCalibration(0);
    private DigitalInput mLimitSwitch = new DigitalInput(1);
    private Encoder getTurretEncoder = new Encoder(0,1);

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

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized double getPosition(){
        return turretEncoder.getRaw();
    }

    @Override
    public void periodic() {
        // Runs every tick
    }

}
