package org.igutech.utils;

/**
 * Created by Kevin on 7/21/2018.
 */

public class FTCMath {

    /**
     * Linear interpolation between a and b by a factor of f
     * @param a Lower bound
     * @param b Upper bound
     * @param f Amount to interpolate between A and B (0.0 - 1.0)
     * @return
     */
    public static double lerp(double a, double b, double f) {
        return a + f * (b - a);
    }

    /**
     * Safer implementation of lerp which guarantees good values even when some are out of range.
     * @param a Lower bound
     * @param b Upper bound
     * @param f Amount to interpolate between A and B (0.0 - 1.0)
     * @return
     * @see #lerp(double, double, double)
     */
    public static double safeLerp(double a, double b, double f) {
        return clamp(a, b, lerp(a, b, clamp(0, 1, f)));
    }

    /**
     *
     * Calculates the motor speed for a trapezoidal motion profile
     *
     * |-----------------|---------------------------------------|-----------------|
     *   startDistance                                              endDistance
     *
     * |_______________________________travelDistance______________________________|
     *
     *
     *                    _______________________________________  ---maxSpeed
     *                  /|                                      |\
     *              /    |                                      |     \
     *          /        |                                      |         \
     *      /            |                                      |             \
     * _/ _ _ _ _ _ _ _ _| _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _| _ _ _ _ _ _ _ _ \_ ---minSpeed
     *
     * @param startDistance    Distance from start position to max speed
     * @param endDistance      Distance from max speed to end
     * @param travelDistance   Total distance to travel (including start and end distance)
     * @param maxSpeed         Maximum speed to reach
     * @param minSpeed         Minimum speed
     * @param currentDistance  Distance already traveled
     * @return                 The speed the motor should be moving
     */
    public static double trapezoidalMotionProfile(double startDistance,
                                                  double endDistance,
                                                  double travelDistance,
                                                  double maxSpeed,
                                                  double minSpeed,
                                                  double currentDistance) {
        if (currentDistance < startDistance) {
            return safeLerp(minSpeed, maxSpeed, currentDistance/startDistance);
        }

        if (currentDistance > travelDistance - endDistance) {
            return safeLerp(maxSpeed, minSpeed, (currentDistance - travelDistance + endDistance)/endDistance);
        }

        return maxSpeed;
    }

    /**
     * Clamp the value v between min and max.
     * @param min Smallest value allowed
     * @param max Largest value allowed
     * @param v   Target value
     * @return
     */
    public static double clamp(double min, double max, double v) {
        if (v < min) return min;
        if (v > max) return max;
        return v;
    }

}
