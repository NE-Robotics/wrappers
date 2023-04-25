package com.northeasternrobotics.wrappers.encoders.ThriftyEncoder;

import com.northeasternrobotics.wrappers.encoders.AbstractSwerveAzmthEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

public class RealThriftyEncoder extends AbstractSwerveAzmthEncoder {

    AnalogInput m_input;
    AnalogEncoder m_encoder;

    double measVoltage;

    public RealThriftyEncoder(int port) {
        m_input = new AnalogInput(port);
        m_encoder = new AnalogEncoder(m_input);
    }

    @Override
    public double getRawAngle_rad() {
        measVoltage = m_input.getVoltage();
        return m_encoder.getAbsolutePosition();
    }


}
