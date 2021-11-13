
package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class Hood extends Subsystem {
    private static Hood mInstance;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private LinearServo left_servo;
    private LinearServo right_servo;

    public static synchronized Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    //Create new Hood subsystem
    private Hood() {
        left_servo = new LinearServo(Constants.kServoAChannel, 5, 7);
        right_servo = new LinearServo(Constants.kServoBChannel, 5, 7);
    }

    public synchronized void setPosition(double setpoint) {
        mPeriodicIO.setpoint = setpoint;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // update current position of hood servos
        left_servo.updateCurPos();
        right_servo.updateCurPos();

        // read position in mm
        mPeriodicIO.left_position = left_servo.getPosition();
        mPeriodicIO.right_position = right_servo.getPosition();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // set setpoints for left and right servos
        left_servo.setPosition(mPeriodicIO.setpoint);
        right_servo.setPosition(mPeriodicIO.setpoint);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Left Servo Position", mPeriodicIO.left_position);
        SmartDashboard.putNumber("Right Servo Position", mPeriodicIO.right_position);
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
        public static double left_position;
        public static double right_position;

        //OUTPUTS
        public double setpoint;
    }

}
