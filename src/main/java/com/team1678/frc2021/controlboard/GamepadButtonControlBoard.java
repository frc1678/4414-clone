package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class GamepadButtonControlBoard {

    private final double kDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private GamepadButtonControlBoard() {
        mController = new CustomXboxController(Constants.kButtonGamepadPort);
    }

    public Rotation2d getJogTurret() {
        double jogX = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        double jogY = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        
        Translation2d mag = new Translation2d(jogX, jogY);
        Rotation2d turret = new Rotation2d(mag.getX(), mag.getY());

        // if (Deadband.inDeadband(mag.norm(), 0.5)) {
        //     return null;
        // }
        return turret;
    }

    public double getJogHood() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        // if (Deadband.inDeadband(jog, kDeadband)) {
        //     return 0.0;
        // }
        return (jog - kDeadband * Math.signum(jog));
    }

    public boolean getIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getReverseIntake() {
        return mController.getTrigger(Side.LEFT);
    }

    public boolean getReverseHopper() {
        return mController.getController().getBumper(Hand.kLeft);
    }

    public boolean getTurnOffIntake() {
        return mController.getController().getBumper(Hand.kRight);
    }

    public boolean getTuck() {
        return mController.getButton(Button.X);
    }

    public boolean getTurretReset() {
        return mController.getController().getBackButtonReleased();
    }

    public boolean getUntuck() {
        return mController.getButton(Button.START);
    }

    public boolean getShoot() {
        return mController.getController().getYButtonPressed();
    }

}