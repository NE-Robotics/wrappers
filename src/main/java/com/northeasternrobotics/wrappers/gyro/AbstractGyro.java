package com.northeasternrobotics.wrappers.gyro;

public abstract class AbstractGyro {
    public abstract void reset();

    public abstract void calibrate();

    public abstract double getRate();

    public abstract double getRawAngle();

    public abstract boolean isConnected();
}
