package com.northeasternrobotics.wrappers.motorcontrollers.REV;


import com.northeasternrobotics.wrappers.motorcontrollers.AbstractSimmableMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;

/**
 * Wrapper for the REV Spark Max motor controller.
 */
public class RealSparkMax extends AbstractSimmableMotorController {

    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    /**
     * Constructs a Spark Max motor controller.
     *
     * @param can_id the CAN ID of the motor controller
     */
    public RealSparkMax(int can_id) {
        m_motor = new CANSparkMax(can_id, MotorType.kBrushless);

        boolean success = false;

        while (!success) {
            var err0 = m_motor.restoreFactoryDefaults();
            var err1 = m_motor.setIdleMode(IdleMode.kCoast);
            var err2 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 19);// Status 0 = Motor output and Faults
            var err3 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 57);// Status 1 = Motor velocity & electrical data
            var err4 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65500);// Status 2 = Motor Position
            var err5 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65500);// Status 3 = Analog Sensor Input
            var err6 = m_motor.setSmartCurrentLimit(40); // 40 A current limit
            success = (err0 == REVLibError.kOk &&
                    err1 == REVLibError.kOk &&
                    err2 == REVLibError.kOk &&
                    err3 == REVLibError.kOk &&
                    err4 == REVLibError.kOk &&
                    err5 == REVLibError.kOk &&
                    err6 == REVLibError.kOk);

            if (!success) {
                System.out.println("Configuration Failed, retrying....");
            }
        }
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

    }

    @Override
    public Object getUnwrappedMotor() {
        return m_motor;
    }

    @Override
    public void setInverted(boolean invert) {
        m_motor.setInverted(invert);
    }

    @Override
    public void setNeutralMode(NeutralMode mode) {
        switch (mode) {
            case UseSavedMode:
            case Coast:
                m_motor.setIdleMode(IdleMode.kCoast);
                break;
            case Brake:
                m_motor.setIdleMode(IdleMode.kBrake);
                break;
        }
    }


    @Override
    public void setClosedLoopGains(double p, double i, double d) {

        // I don't know why we need to do this, but it makes sim line up with real life.
        p /= 1000;

        //Convert to Rev units of RPM
        p = Units.radiansPerSecondToRotationsPerMinute(p);
        i = Units.radiansPerSecondToRotationsPerMinute(i);
        d = Units.radiansPerSecondToRotationsPerMinute(d);

        m_pidController.setP(p);
        m_pidController.setI(i);
        m_pidController.setD(d);
        m_pidController.setOutputRange(-1.0, 1.0);
    }


    @Override
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        m_pidController.setReference(Units.radiansPerSecondToRotationsPerMinute(velocityCmd_radpersec),
                CANSparkMax.ControlType.kVelocity,
                0,
                arbFF_V,
                SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void overrideDefaultNativeUnitsPerRotation(double nativeUnitsPerRotation) {
        System.out.println("Spark Max: Native Units Per Rotation Does Not Apply");
    }


    @Override
    public void setVoltageCmd(double cmd_v) {
        m_motor.setVoltage(cmd_v);
    }


    @Override
    public double getCurrent_A() {
        return m_motor.getOutputCurrent();
    }


    @Override
    public double getVelocity_radpersec() {
        return Units.degreesToRadians(m_encoder.getVelocity() * 360 / 60.0);
    }


    @Override
    public void follow(Object leader) {
        if (leader.getClass() == RealSparkMax.class) {
            this.m_motor.follow(((RealSparkMax) leader).m_motor);
        } else {
            throw new IllegalArgumentException(leader.getClass().toString() + " cannot be followed by a " + this.getClass().toString());
        }

    }

    @Override
    public double getPosition_rad() {
        return Units.rotationsToRadians(m_encoder.getPosition());
    }


    @Override
    public double getAppliedVoltage_V() {
        return m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    }

    @Override
    public void resetDistance() {
        m_encoder.setPosition(0.0);
    }


}
