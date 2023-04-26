package com.northeasternrobotics.wrappers.gyro;

import com.northeasternrobotics.wrappers.HardwareWrapper;
import com.northeasternrobotics.wrappers.SimDeviceBanks;

/**
 * Simulated Gyro Wrapper
 */
public class SimGyro extends AbstractGyro {
    double rate; // in rad/sec
    double angle; // in rad

    /**
     * Constructor for SimGyro
     */
    public SimGyro() {
        SimDeviceBanks.addSPIDevice(this, 0); // TODO are we actually on CS 0?
    }

    @Override
    public void reset() {
        rate = 0;
        angle = 0;
    }

    @Override
    public void calibrate() {
        //nothing to do
        System.out.println("Sim Gyro Calibration Completed!");
    }

    @Override
    public double getRate() {
        return rate;
    }

    @Override
    public double getRawAngle() {
        return angle;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    /**
     * Simulates the gyro updating
     * @param newRate_radpersec the new rate of the gyro in rad/sec
     */
    public void simUpdate(double newRate_radpersec) {
        rate = newRate_radpersec; //Sim gyro is inverted
        angle += newRate_radpersec * HardwareWrapper.k_hardwareSimLoopSeconds;
    }

    /**
     * Simulates the gyro setting the angle
     * @param newAngle_rad the new angle of the gyro in rad
     */
    public void simSetAngle(double newAngle_rad) {
        rate = 0;
        angle = newAngle_rad;
    }
}
