package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.team1678.frc2021.Constants;

public class ServoSubsystem implements Subsystem {
    private Servo hoodServo;
    public ServoSubsystem() {
        hoodServo = new Servo(Constants.kServo);
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
