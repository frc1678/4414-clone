
package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Servo;

import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends LinearServo {
    private static Hood mInstance;
    private boolean mHoming = true;
    private Servo hoodServoA;
    private Servo hoodServoB;

    //Create new Hood subsystem
    public Hood() {
        super(mInstance.getChannel(), mInstance.hashCode(), mInstance.getRaw());
        super.getChannel();
        hoodServoA = new Servo(Constants.HOOD_SERVO_A);
        hoodServoB = new Servo(Constants.HOOD_SERVO_B);
    }

    public void hoodExtend() {
        hoodServoA.set(Constants.HOOD_SPEED);
        hoodServoB.set(Constants.HOOD_SPEED);
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
            System.out.println("Homed!!!");
            mHoming = false;
        }
        super.readPeriodicInputs();
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean("Calibrated", !mHoming);
        SmartDashboard.putBoolean("Hood at Homing Location", atHomingLocation());
    }

}
