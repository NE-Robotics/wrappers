package com.northeasternrobotics.wrappers.encoders;

import com.northeasternrobotics.wrappers.encoders.CANCoder.RealCANCoder;
import com.northeasternrobotics.wrappers.encoders.SRXEncoder.RealSRXEncoder;
import com.northeasternrobotics.wrappers.encoders.Sim.SimSwerveAzmthEncoder;
import com.northeasternrobotics.wrappers.encoders.ThriftyEncoder.RealThriftyEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Wrapper for swerve azimuth encoders / absolute through bore encoders
 */
public class WrapperedSwerveAzimuthEncoder {
    AbstractSwerveAzmthEncoder enc;
    double curAngleRad;
    double mountingOffset;

    /**
     * Constructor for a wrapped swerve azimuth encoder, or general through bore encoder
     *
     * @param type                   Type of swerve azimuth/absolute through bore encoder to use
     * @param prefix                 Prefix for the encoder
     * @param id                     ID of the encoder
     * @param dfltMountingOffset_rad Default mounting offset of the encoder in radians
     */
    public WrapperedSwerveAzimuthEncoder(SwerveAzmthEncType type, String prefix, int id, double dfltMountingOffset_rad) {
        if (RobotBase.isReal()) {
            switch (type) {
                case SRXEncoder:
                    //ID = digital input
                    enc = new RealSRXEncoder(id);
                    break;
                case CANCoder:
                    //ID = CAN ID
                    enc = new RealCANCoder(id);
                    break;
                case Thrifty:
                    //ID = Analog Input
                    enc = new RealThriftyEncoder(id);
                    break;
            }
        } else {
            enc = new SimSwerveAzmthEncoder(id);
        }
        this.mountingOffset = dfltMountingOffset_rad;
    }

    /**
     * @return Motor controller object
     */
    public Object getUnwrappedEncoder() {
        return enc.getUnwrappedEncoder();
    }

    /**
     * Updates the abstracted swerve azimuth encoder values
     */
    public void update() {
        curAngleRad = Units.degreesToRadians(wrapAngleDeg(Units.radiansToDegrees(enc.getRawAngle_rad() - mountingOffset)));
    }

    /**
     * @return Angle of the swerve azimuth encoder in radians
     */
    public double getAngle_rad() {
        return curAngleRad;
    }

    /**
     * @return Angle of the swerve azimuth encoder in degrees
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getAngle_rad());
    }

    private double wrapAngleDeg(double angle) {
        angle %= 360;
        angle = angle > 180 ? angle - 360 : angle;
        angle = angle < -180 ? angle + 360 : angle;
        return angle;
    }

    /**
     * Encoder types for real robot
     */
    public enum SwerveAzmthEncType {
        /**
         * CTRE SRX Mag Encoder
         */
        SRXEncoder,
        /**
         * CTRE CanCoder
         */
        CANCoder,
        /**
         * Thrifty bot swerve encoder
         */
        Thrifty
    }


}
