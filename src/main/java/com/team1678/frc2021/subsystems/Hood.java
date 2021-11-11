
package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class Hood extends LinearServo {
    private static Hood mInstance;

    private Encoder mEncoder;
    private boolean mHoming = true;
    private Servo hoodServoA;
    private Servo hoodServoB;

    public static synchronized Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    //Create new Hood subsystem
    private Hood() {
        super(mInstance.getChannel(), mInstance.hashCode(), mInstance.getRaw());
        hoodServoA = new Servo(Constants.kHoodConstants.kHoodServoA);
        hoodServoB = new Servo(Constants.kHoodConstants.kHoodServoB);
        Encoder encoder = new Encoder(0,1);
    }

    public void hoodExtend() {
        hoodServoA.set(Constants.kHoodConstants.kHoodSpeed);
        hoodServoB.set(Constants.kHoodConstants.kHoodSpeed);
    }

    public void hoodStop() {
        hoodServoA.set(0);
        hoodServoB.set(0);
    }

    public synchronized boolean atHomingLocation() {
        return false;
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized double getAtGoal() {
        return getAngle();
    }

    public synchronized double getTucked() {
        return getAngle();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mHoming && atHomingLocation()) {
            System.out.println("is homing");
            mEncoder.reset();
            System.out.println("Homed!!!");
            mHoming = false;
        }
        SmartDashboard.putNumber(("Hood Encoder Readout"), mEncoder.getDistance());
        super.readPeriodicInputs();
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean("Calibrated", !mHoming);
        SmartDashboard.putBoolean("Hood at Homing Location", atHomingLocation());
    }

}
