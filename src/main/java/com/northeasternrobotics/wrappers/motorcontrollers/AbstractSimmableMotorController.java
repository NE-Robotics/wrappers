package com.northeasternrobotics.wrappers.motorcontrollers;

public abstract class AbstractSimmableMotorController {
    public abstract void setInverted(boolean invert);

    //Velocity Closed-loop gains need to be in units of Volts, rad/sec of error, and seconds (for integration/div)
    public abstract void setClosedLoopGains(double p, double i, double d);

    public abstract void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V);

    public abstract void setVoltageCmd(double cmd_v);

    public abstract void resetDistance();

    public abstract double getCurrent_A();

    public abstract double getVelocity_radpersec();

    public abstract double getPosition_rad();

    public abstract double getAppliedVoltage_V();

    public abstract void follow(Object leader);
    public abstract void overrideDefaultNativeUnitsPerRotation(double nativeUnitsPerRotation);
    public abstract void setNeutralMode(NeutralMode mode);
    public enum NeutralMode {
        /** Use the NeutralMode that is set in the motor controller's persistent storage. */
        UseSavedMode(0),
        /** When commanded to neutral, motor leads are set to high-impedance, allowing mechanism to coast. */
        Coast(1),
        /** When commanded to neutral, motor leads are commonized electrically to reduce motion. */
        Brake(2);

        /**
         * Value of NeutralMode
         */
        public int value;
        /**
         * Create NeutralMode from specified value
         * @param value Value of NeutralMode
         */
        NeutralMode(int value)
        {
            this.value = value;
        }
    };
}
