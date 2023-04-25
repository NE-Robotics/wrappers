package com.northeasternrobotics.wrappers.encoders.CANCoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.northeasternrobotics.wrappers.encoders.AbstractSwerveAzmthEncoder;

public class RealCANCoder extends AbstractSwerveAzmthEncoder {

    CANCoder cancoder;

    double supplyVoltage;

    boolean magnetGood;

    boolean magnetFaulted;

    public RealCANCoder(int can_id) {

        cancoder = new CANCoder(can_id);

        cancoder.configFactoryDefault();

        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(config);
    }

    @Override
    public double getRawAngle_rad() {
        supplyVoltage = cancoder.getBusVoltage();

        var tmp = cancoder.getMagnetFieldStrength();
        magnetGood = (tmp == MagnetFieldStrength.Good_GreenLED);
        magnetFaulted = (tmp == MagnetFieldStrength.Invalid_Unknown || tmp == MagnetFieldStrength.BadRange_RedLED);

        return cancoder.getAbsolutePosition();
    }


}
