package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.team1678.frc2021.Constants;

public class ServoSubsystem implements Subsystem {
    private Servo hoodServo;
    private Encoder hoodEncoder;
    public ServoSubsystem() {
        hoodServo = new Servo(Constants.kServo);
    }

    public static class ServoSubsystemConstants {
        public int kHoodServoA = 2;
        public int kHoodServoB = 3;
        public int kHoodSpeed = 1;

        public double kHoodRadius = 11.904; // radius of hood
        public int kAbsoluteEncoderID =  3;

    }

    public void ExtendServo() {
        hoodServo.set(Constants.kServoExtend);
    }

    public void RetractServo() {
        hoodServo.set(Constants.kServoRetract);
    }

    @Override
    public void periodic(){
    }
}
