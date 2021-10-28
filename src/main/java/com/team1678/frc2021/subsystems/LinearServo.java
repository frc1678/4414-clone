package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.io.FileInputStream;

public class LinearServo extends Servo {

    double m_speed;
    double m_length;

    double setPos;
    double curPos;

/**
 * Parameters for L 16-R Actuonix Linear Actuators
 *
 * @param channel PWM channel used to control the servo
 * @param length max length of the servo [mm]
 * @parom speed max speed of the servo [mm/sec]
 **/

public LinearServo(int channel, int length, int speed){
    super(channel);
    setBounds(2.0,1.8,1.5, 1.2, 1.0);
    m_length = length;
    m_speed = speed;
}


    /**
 * Run this method in any periodic funtion to update the position estimation servo
 *
 * @param setpoint the tartget position of servo [mm]
 */

public void setPosition(double setpoint){
    setPos = MathUtil.clamp(setpoint,0,m_length);
    setSpeed((setPos/m_length*2)-1);
}

double lastTime = 0;
/**
 * Run this method in any periodic function to update the position estimation of the servo
 */
public void updateCurPos(){
    double dt = Timer.getFPGATimestamp() - lastTime;
    if(curPos > setPos + m_speed*dt){
        curPos -= m_speed*dt;
    }else if(curPos < setPos - m_speed*dt){
        curPos += m_speed*dt;
    }else {
        curPos = setPos;
    }
}

/**
 * Current position of the servo, must be callin {@link #updateCurPos() updateCurPos()} periodically
 *
 * @return Servo Position [mm]
 */
public double getPosition(){
    return curPos;
}
/**
 * Checks is the servo is at its target position, must be calling {@link #updateCurPos() updateCurPos()} periodically
 * @return true when servo is at its target
 */
public boolean isFinished(){
    return curPos == setPos;
}

    protected void outputTelemetry() {
    }

    protected void writePeriodicOutputs() {
    }

    protected void readPeriodicInputs() {
    }
}
