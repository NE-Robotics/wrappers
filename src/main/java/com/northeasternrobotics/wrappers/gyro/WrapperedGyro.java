package com.northeasternrobotics.wrappers.gyro;

import com.northeasternrobotics.wrappers.gyro.ADXRS453.RealADXRS453;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Wrapper for gyroscopes
 */
public class WrapperedGyro {
    AbstractGyro gyro;
    double offset_rad = 0;
    private double curAngle_rad;

    /**
     * Constructor for a wrapped gyroscope
     *
     * @param type Type of gyroscope to use
     */
    public WrapperedGyro(GyroType type) {
        if (RobotBase.isReal()) {
            if (type == GyroType.ADXRS453) {
                gyro = new RealADXRS453();
            } else if (type == GyroType.NAVX) {

            }
        } else {
            gyro = new SimGyro();
        }
    }

    /**
     * Updates the abstracted gyro values
     */
    public void update() {
        // Gyros are inverted in reference frame (positive clockwise)
        // and we maintain our own offset in code when rezeroing.
        curAngle_rad = gyro.getRawAngle() * -1.0 + offset_rad;
    }

    /**
     * Resets the gyro to a given angle
     *
     * @param curAngle_rad Angle to reset the gyro to
     */
    public void reset(double curAngle_rad) {
        offset_rad = curAngle_rad;
        gyro.reset();
    }

    /**
     * Calibrates the gyro, most be still for this to work
     */
    public void calibrate() {
        gyro.calibrate();
    }

    /**
     * @return Rate of the gyro in radians per second
     */
    public double getRate_radpersec() {
        return gyro.getRate();
    }

    /**
     * @return Current angle of the gyro in radians
     */
    public double getAngle_rad() {
        return curAngle_rad;
    }

    /**
     * @return Weather or not the gyro is connected
     */
    public boolean isConnected() {
        return gyro.isConnected();
    }

    /**
     * @return The rotation2d object of the gyro
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getAngle_rad());
    }

    /**
     * Enum for the type of gyroscope to use
     */
    public enum GyroType {
        /**
         * The ADXRS453 gyroscope, standard SPI plugin
         */
        ADXRS453,
        /**
         * NavX gyroscope, on the MXP port
         */
        NAVX
    }

}
