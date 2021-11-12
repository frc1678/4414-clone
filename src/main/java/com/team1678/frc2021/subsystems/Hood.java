
package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class Hood extends Subsystem {
    private static Hood mInstance;

    private LinearServo hoodServoA;
    private LinearServo hoodServoB;

    public static synchronized Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    //Create new Hood subsystem
    private Hood() {
        hoodServoA = new LinearServo(0, 5, 7);
        hoodServoB = new LinearServo(1, 5, 7);
    }

    public void hoodExtend() {
        hoodServoA.set(7);
        hoodServoB.set(7);
    }

    public void setPosition(double position) {
        hoodServoA.setPosition(position);
        hoodServoB.setPosition(position);
    }

    public void hoodStop() {
        hoodServoA.set(0);
        hoodServoB.set(0);
    }

    public synchronized double getAngle() {
        return hoodServoA.getPosition();
    }

    public synchronized double getAtGoal() {
        return getAngle();
    }

    public synchronized double getTucked() {
        return getAngle();
    }

    @Override
    public synchronized void readPeriodicInputs() {
    }

    @Override
    public void outputTelemetry() {
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
