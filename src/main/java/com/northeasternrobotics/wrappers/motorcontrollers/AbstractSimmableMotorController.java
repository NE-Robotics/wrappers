package com.northeasternrobotics.wrappers.motorcontrollers;

/**
 * Abstract class for a Simulateable Motor Controller.
 */
public abstract class AbstractSimmableMotorController {
    /**
     * @return the wrapped motor controller object
     */
    public abstract Object getUnwrappedMotor();

    /**
     * @param invert if set to true inverts the motors direction
     */
    public abstract void setInverted(boolean invert);

    /**
     * Velocity Closed-loop gains need to be in units of Volts, rad/sec of error, and seconds (for integration/div)
     *
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public abstract void setClosedLoopGains(double p, double i, double d);

    /**
     * PID based velocity control
     *
     * @param velocityCmd_radpersec desired velocity in radians per second
     * @param arbFF_V               arbitrary feed forward voltage
     */
    public abstract void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V);

    /**
     * Voltage based control
     *
     * @param cmd_v desired voltage
     */
    public abstract void setVoltageCmd(double cmd_v);

    /**
     * Resets the encoder position to zero.
     */
    public abstract void resetDistance();

    /**
     * @return the current draw of the motor controller in Amps
     */
    public abstract double getCurrent_A();

    /**
     * @return the current velocity of the motor in radians per second
     */
    public abstract double getVelocity_radpersec();

    /**
     * @return the current position of the motor in radians
     */
    public abstract double getPosition_rad();

    /**
     * @return the current applied voltage of the motor controller in Volts
     */
    public abstract double getAppliedVoltage_V();

    /**
     * Sets a motor controller to follow another of the same type
     * NOTE: Improvised cross controller following will be added in the future
     * but is not officially supported by vendors and may have odd behaviour
     *
     * @param leader the lead controller to follow
     */
    public abstract void follow(Object leader);

    /**
     * @param nativeUnitsPerRotation the native units per rotation of the encoder,
     *                               this is used of sensor readings aren't based on
     *                               the device standard or to compensate for a gearbox
     */
    public abstract void overrideDefaultNativeUnitsPerRotation(double nativeUnitsPerRotation);

    /**
     * @param mode the neutral mode to set the motor controller to, this
     *             is a cross controller implementation but UseSaveMode
     *             is not supported by all vendors and defaults to Coast
     *             if not supported
     */
    public abstract void setNeutralMode(NeutralMode mode);

    /**
     * the neutral mode of the motor controller: Break, Coast, or UseSavedMode (EEPROM)
     */
    public enum NeutralMode {
        /**
         * Use the NeutralMode that is set in the motor controller's persistent storage.
         */
        UseSavedMode(0),
        /**
         * When commanded to neutral, motor leads are set to high-impedance, allowing mechanism to coast.
         */
        Coast(1),
        /**
         * When commanded to neutral, motor leads are commonized electrically to reduce motion.
         */
        Brake(2);

        /**
         * Value of NeutralMode
         */
        public int value;

        /**
         * Create NeutralMode from specified value
         *
         * @param value Value of NeutralMode
         */
        NeutralMode(int value) {
            this.value = value;
        }
    }
}
