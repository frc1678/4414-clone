package com.team1678.frc2021.controlboard;

import com.team254.lib.geometry.Rotation2d;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    
    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final GamepadButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    public void reset() {
    }
 
    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }

    public boolean getReverseIntake() {
        return mButtonControlBoard.getReverseIntake();
    }

    public boolean getVisionAim() {
        return mButtonControlBoard.getVisionAim();
    }

    public double getJogHood() {
        return mButtonControlBoard.getJogHood();
    }

    public boolean getWantHoodScan() {
        return mButtonControlBoard.getWantHoodScan();
    }

    public boolean getWantResetShooter() {
        return mButtonControlBoard.getShooterReset();
    }

    public int getManualHoodSet() {
        return mButtonControlBoard.getHoodManualSet();
    }

    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    public boolean getPreShot() {
        return mButtonControlBoard.getPreShot();
    }

    public boolean getTuck() {
        return mButtonControlBoard.getTuck() /*|| mDriveControlBoard.getTuck()*/;
    }

    public boolean getUntuck() {
        return mButtonControlBoard.getUntuck();
    }

    public boolean getTestSpit() {
        return mButtonControlBoard.getTestSpit();
    }

    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    // climber
    public boolean getClimbMode() {
        return mButtonControlBoard.climbMode();
    }

    public boolean getLeaveClimbMode() {
        return mButtonControlBoard.getLeaveClimbMode(); 
    }

    public int getClimberJog() {
        return mButtonControlBoard.getClimberJog();
    }

    public Rotation2d getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }
}
