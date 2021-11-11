
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
    private Hood(){
        super(mInstance.getChannel(), mInstance.hashCode(), mInstance.getRaw());
        setBounds(2.0,1.8,1.5, 1.2, 1.0);
        m_length = 5;
        m_speed = 7;
        hoodServoA = new LinearServo(0,5,7);
        hoodServoB = new LinearServo(1,5,7);
    }

    public void hoodExtend() {
        hoodServoA.setSpeed(7);
        hoodServoB.setSpeed(7);
    }

    public void hoodStop() {
        hoodServoA.setSpeed(0);
        hoodServoB.setSpeed(0);
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
    public void updateCurPos() {
        super.updateCurPos();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        updateCurPos();
        }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean("Calibrated", true);
        SmartDashboard.putBoolean("Hood at Detected Location", isFinished());
    }

}
