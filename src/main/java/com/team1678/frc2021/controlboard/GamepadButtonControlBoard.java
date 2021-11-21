package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

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

    public double getJogHood() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public boolean getWantHoodScan() {
        return mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getTestSpit() {
        return mController.getController().getStickButtonReleased(Hand.kRight);
    }

    public void setRumble(boolean on) { //TODO: all 5 power cells indexed
        mController.setRumble(on);
    }

    public boolean getSpinUp() {
        return false;
        // return mController.getController().getAButtonPressed();
    }

    public boolean getTuck() {
        return mController.getButton(Button.X);
    }

    public boolean getUntuck() {
        return mController.getButton(Button.START);
    }

    public int getHoodManualSet() {
        int pov_read = mController.getController().getPOV();
        switch(pov_read){
            case 0:
                return 1;
            case 180:
                return -1;
            default:
                return 0;
        }
    }

    public boolean getShoot() {
        return mController.getController().getYButtonPressed();
    }

    public boolean getPreShot() {
        return mController.getController().getAButtonPressed();
    }
    
    public boolean getIntake() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    
    public boolean getShooterReset() {
        return mController.getButton(Button.B);
    }

    // Intake
    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getReverseIntake() {
        return mController.getTrigger(Side.LEFT);
    }

    public boolean getVisionAim() {
        return mController.getController().getBumper(Hand.kRight);
    }

    // climber
    public boolean climbMode() {
        return mController.getButton(CustomXboxController.Button.LB) && mController.getButton(CustomXboxController.Button.RB)  && 
        mController.getTrigger(CustomXboxController.Side.LEFT) &&  mController.getTrigger(CustomXboxController.Side.RIGHT);
    }

    public boolean getLeaveClimbMode() {
        return mController.getButton(Button.BACK) && mController.getButton(Button.START);
    }

    public int getClimberJog(){
        int povread = mController.getController().getPOV();
        switch(povread){
            case 0:
                return 1;
            case 180:
                return -1;
            default:
                return 0;
        }
    }

    public Rotation2d getJogTurret() {
        double jogX = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        double jogY = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        
        Translation2d mag = new Translation2d(jogX, jogY);
        Rotation2d turret = mag.direction();

        if (Deadband.inDeadband(mag.norm(), 0.5)) {
            return null;
        }
        return turret;
    }

}
