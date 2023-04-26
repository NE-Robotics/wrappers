package com.northeasternrobotics.wrappers.gyro;

/**
 * Abstract class for a gyro.
 */
public abstract class AbstractGyro {
    /**
     * Resets the gyro to a heading of zero.
     */
    public abstract void reset();

    /**
     * Calibrates the gyro.
     */
    public abstract void calibrate();

    /**
     * @return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
     */
    public abstract double getRate();

    /**
     * @return the angle of the gyro in radians
     */
    public abstract double getRawAngle();

    /**
     * @return true if the gyro is connected else false
     */
    public abstract boolean isConnected();
}
