package com.team2910.lib.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1678.frc2021.Ports;
import com.team2910.lib.math.Rotation2;

public class Pigeon extends Gyroscope {
    private final PigeonIMU pigeon;

    public Pigeon(int canId) {
        pigeon = new PigeonIMU(canId);
    }

    public PigeonIMU getPigeonIMU() {
        return pigeon;
    }

    private static Pigeon instance = null;
    public static Pigeon getInstance(){
		if(instance == null){
			instance = new Pigeon(Ports.PIGEON_TALON);
		}
		return instance;
	}

    @Override
    public void calibrate() {
        pigeon.setFusedHeading(0);
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    @Override
    public double getUnadjustedRate() {
        return 0; // TODO
    }

    public double getAxis(Axis axis) {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        switch (axis) {
            case PITCH:
                return Math.toRadians(ypr[1]);
            case ROLL:
                return Math.toRadians(ypr[2]);
            case YAW:
                return Math.toRadians(ypr[0]);
            default:
                return 0.0;
        }
    }

    public void setAngle(double angle){
		pigeon.setFusedHeading(angle * 64.0, 10);
		pigeon.setYaw(angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}