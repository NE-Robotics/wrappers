package com.northeasternrobotics.wrappers.encoders.Sim;

import com.northeasternrobotics.wrappers.SimDeviceBanks;
import com.northeasternrobotics.wrappers.encoders.AbstractSwerveAzmthEncoder;

/**
 * Simulated Swerve Azmth Encoder Wrapper
 */
public class SimSwerveAzmthEncoder extends AbstractSwerveAzmthEncoder {

    double curAngle_rad;

    double STEPS_PER_REV = 4096.0; //Simulate quantization

    /**
     * Constructor for SimSwerveAzmthEncoder
     *
     * @param port the DIO port the encoder is plugged into
     */
    public SimSwerveAzmthEncoder(int port) {
        SimDeviceBanks.addDIDevice(this, port);
    }

    /**
     * Simulates the encoder updating
     *
     * @param curAngle_rad the new angle of the encoder in rad
     */
    public void setRawAngle(double curAngle_rad) {
        this.curAngle_rad = curAngle_rad;
    }

    @Override
    public double getRawAngle_rad() {
        return Math.round(curAngle_rad * STEPS_PER_REV / 2 / Math.PI) * 2 * Math.PI / STEPS_PER_REV;
    }

}
