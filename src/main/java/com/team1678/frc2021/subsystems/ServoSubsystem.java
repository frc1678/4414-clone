package com.team1678.frc2021.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.team1678.frc2021.Constants;

public class ServoSubsystem implements Subsystem {
    private Servo hoodServo;
    public ServoSubsystem() {
        hoodServo = new Servo(Constants.SERVO);
    }

    public void ExtendServo() {
        hoodServo.set(Constants.SERVO_EXTEND);
    }

    public void RetractServo() {
        hoodServo.set(Constants.SERVO_RETRACT);
    }

    @Override
    public void periodic(){
    }
}
