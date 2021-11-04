package com.team1678.frc2021.lib.util;

/**
 * Basic Utility Methods
 */
public class Util {

    public static final double K_EPSILON = 1e-12;

    private Util() {
        // Private utility method
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, K_EPSILON);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static double normalize(double current, double test){
        return Math.max(current, test);
    }

    public static double deadBand(double val, double deadBand){
        return (Math.abs(val) > Math.abs(deadBand)) ? val : 0.0;
    }
}
