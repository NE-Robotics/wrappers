package com.northeasternrobotics.wrappers.encoders;

/**
 * Abstract class for a swerve azimuth encoder.
 */
public abstract class AbstractSwerveAzmthEncoder {
    /**
     * @return the angle of the encoder in radians
     */
    public abstract double getRawAngle_rad();

    /**
     * @return The unwrapped encoder object
     */
    public abstract Object getUnwrappedEncoder();

}
