package com.team1678.frc2021.controlboard;

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

    public boolean getIntake() {
        return mButtonControlBoard.getIntake();
    }

    public boolean getReverseIntake() {
        return mButtonControlBoard.getReverseIntake();
    }

    // public boolean getReverseHopper() {
    //     return mButtonControlBoard.getReverseHopper();
    // }

    public boolean getTurnOffIntake() {
        return mButtonControlBoard.getTurnOffIntake();
    }

    public boolean getTuck() {
        return mButtonControlBoard.getTuck();
    }
    
    public boolean getTurretReset() {
        return mButtonControlBoard.getTurretReset();
    }

    public boolean getUntuck() {
        return mButtonControlBoard.getUntuck();
    }

    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    public boolean climbMode() {
        return mButtonControlBoard.climbMode();
    }

    public int getClimberJog() {
        return mButtonControlBoard.getClimberJog();
    }
}
