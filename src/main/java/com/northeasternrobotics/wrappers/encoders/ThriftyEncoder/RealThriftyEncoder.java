package com.northeasternrobotics.wrappers.encoders.ThriftyEncoder;

import com.northeasternrobotics.wrappers.encoders.AbstractSwerveAzmthEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Wrapper for the ThriftyEncoder
 */
public class RealThriftyEncoder extends AbstractSwerveAzmthEncoder {

    AnalogInput m_input;
    AnalogEncoder m_encoder;

    double measVoltage;

    /**
     * Constructor for ThriftyEncoder
     *
     * @param port the DIO port the encoder is plugged into
     */
    public RealThriftyEncoder(int port) {
        m_input = new AnalogInput(port);
        m_encoder = new AnalogEncoder(m_input);
    }

    @Override
    public Object getUnwrappedEncoder() {
        return m_encoder;
    }

    @Override
    public double getRawAngle_rad() {
        measVoltage = m_input.getVoltage();
        return m_encoder.getAbsolutePosition();
    }


}
