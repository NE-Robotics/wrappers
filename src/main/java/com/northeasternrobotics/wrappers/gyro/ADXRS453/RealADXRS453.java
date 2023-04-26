package com.northeasternrobotics.wrappers.gyro.ADXRS453;

import com.northeasternrobotics.wrappers.gyro.AbstractGyro;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * ADXRS453 Wrappered gyroscope implementation
 */
public class RealADXRS453 extends AbstractGyro {
    /**
     * The gyro object used
     */
    ADXRS450_Gyro realGyro;

    /**
     * Gyro constructor, uses Onboard CS0 port
     */
    public RealADXRS453() {
        realGyro = new ADXRS450_Gyro(Port.kOnboardCS0);
        realGyro.calibrate();
    }

    @Override
    public void reset() {
        realGyro.reset();
    }

    @Override
    public void calibrate() {
        System.out.println("== GYRO: CALIBRATION IN PROCESS, DO NOT MOVE ROBOT...");
        realGyro.calibrate();
        System.out.println("== ... Complete!");
    }

    @Override
    public double getRate() {
        return Units.degreesToRadians(realGyro.getRate());
    }

    @Override
    public double getRawAngle() {
        return Units.degreesToRadians(realGyro.getAngle());
    }

    @Override
    public boolean isConnected() {
        return realGyro.isConnected();
    }

}
