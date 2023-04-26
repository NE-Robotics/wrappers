package com.northeasternrobotics.wrappers.motorcontrollers.Sim;

import com.northeasternrobotics.wrappers.HardwareWrapper;
import com.northeasternrobotics.wrappers.SimDeviceBanks;
import com.northeasternrobotics.wrappers.motorcontrollers.AbstractSimmableMotorController;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;

/**
 * CTRE doesn't currently support simulating the internal functionality of a TalonFX.
 * We provide a wrapper and interface abstraction here to enable simulation through
 * a transparent wrapper layer in robot code.
 */
public class SimSmartMotor extends AbstractSimmableMotorController {

    final double CURRENT_LIM_I_GAIN = 0.02;
    /**
     * List of followers
     */
    public ArrayList<SimSmartMotor> simFollowers = new ArrayList<SimSmartMotor>();
    boolean isInverted;
    double kP;
    double kI;
    double kD;
    double curLimitFactor = 1.0;
    double velErr_accum;
    double velErr_prev;
    private double curWindingVoltage;
    private double curCurrent;
    private double curVel_radpersec;
    private double curSupplyVoltage = 12.0;
    private double curPos_rad;

    /**
     * Constructor for SimSmartMotor
     *
     * @param can_id the CAN ID of the SimSmartMotor
     */
    public SimSmartMotor(int can_id) {
        SimDeviceBanks.addCANDevice(this, can_id);
    }

    @Override
    public void setInverted(boolean invert) {
        isInverted = invert;
    }

    @Override
    public void setNeutralMode(NeutralMode mode) {
        System.out.println("SimSmartMotor: Neutral Modes Not Implemented");
    }

    @Override
    public void setClosedLoopGains(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    @Override
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        setVoltageCmd(pidSim(velocityCmd_radpersec, arbFF_V));
    }

    @Override
    public void setVoltageCmd(double cmd_v) {
        curWindingVoltage = limitVoltage(cmd_v) * (isInverted ? -1.0 : 1.0);
        for (AbstractSimmableMotorController follower : simFollowers) {
            follower.setVoltageCmd(curWindingVoltage);
        }
    }

    @Override
    public void overrideDefaultNativeUnitsPerRotation(double nativeUnitsPerRotation) {
        System.out.println("SimSmartMotor: Native Units Per Rotation Only Impacts Real Motors");
    }

    // Returns the current draw of the motor controller from the rest of the electrical system.
    @Override
    public double getCurrent_A() {
        return curCurrent;
    }

    @Override
    public double getVelocity_radpersec() {
        return curVel_radpersec * (isInverted ? -1.0 : 1.0);
    }

    @Override
    public double getPosition_rad() {
        return curPos_rad * (isInverted ? -1.0 : 1.0);
    }

    /**
     * Sets the true velocity of the motor in radians per second
     *
     * @param velocity_radpersec velocity of the motor in radians per second
     */
    public void sim_setActualVelocity(double velocity_radpersec) {
        curVel_radpersec = velocity_radpersec;
        curPos_rad += curVel_radpersec * HardwareWrapper.k_hardwareSimLoopSeconds;
    }

    /**
     * Sets the true position of the motor in radians
     *
     * @param pos_rad position of the motor in radians
     */
    public void sim_setActualPosition(double pos_rad) {
        curVel_radpersec = (pos_rad - curPos_rad) / HardwareWrapper.k_hardwareSimLoopSeconds;
        curPos_rad = pos_rad;
    }

    /**
     * Get the voltage applied to the motor windings
     *
     * @return The voltage applied to the motor windings
     */
    public double sim_getWindingVoltage() {
        return curWindingVoltage * curLimitFactor;
    }

    /**
     * Set the supply voltage to the motor controller
     *
     * @param supply_V The supply voltage to the motor controller
     */
    public void sim_setSupplyVoltage(double supply_V) {
        curSupplyVoltage = supply_V;
    }

    /**
     * Set the current flowing through the motor windings
     *
     * @param cur_A The current flowing through the motor windings
     */
    public void sim_setCurrent(double cur_A) {
        curCurrent = cur_A * Math.signum(curSupplyVoltage); //H bridge will reverse current flow
    }

    private double limitVoltage(double in) {
        if (in > curSupplyVoltage) {
            return curSupplyVoltage;
        } else if (in < -curSupplyVoltage) {
            return -curSupplyVoltage;
        } else {
            return in;
        }
    }

    /**
     * A rough approximation of the current limiting behavior of a real smart motor controllers
     */
    public void sim_updateCurrentLimit() {
        // whelp. Super rough approximation of a current limit. Just an I gain on
        // whether we're above the current limit. Should be updated faster
        // at the sim ts rate

        double err = 40.0 - Math.abs(curCurrent);
        curLimitFactor += HardwareWrapper.k_hardwareSimLoopSeconds * CURRENT_LIM_I_GAIN * err;

        if (curLimitFactor > 1.0) {
            curLimitFactor = 1.0;
        }

        if (curLimitFactor < 0.0) {
            curLimitFactor = 0.0;
        }

    }

    /**
     * A rough guess at the behavior of the closed loop controllers on the smart motor controllers
     */
    private double pidSim(double vel_cmd, double arb_ff_V) {

        var velError_RPM = Units.radiansPerSecondToRotationsPerMinute(vel_cmd - getVelocity_radpersec());

        // Simulate sensor dropout... or just the whole "close enough to zero" behavior
        if (Math.abs(velError_RPM) < 5.0) {
            velError_RPM = 0;
        }

        velErr_accum += velError_RPM;

        var velErr_delta = (velError_RPM - velErr_prev) / HardwareWrapper.k_periodicLoopSeconds;

        var pTerm = velError_RPM * kP;
        var dTerm = velErr_delta * kD;
        var iTerm = velErr_accum * kI;

        velErr_prev = velError_RPM;

        return limitVoltage((pTerm + dTerm + iTerm) * curSupplyVoltage + arb_ff_V);
    }


    @Override
    public void follow(Object leader) {
        if (leader.getClass() == SimSmartMotor.class) {
            ((SimSmartMotor) leader).simFollowers.add(this);
        } else {
            throw new IllegalArgumentException(leader.getClass().toString() + " cannot be followed by a " + this.getClass().toString());
        }

    }

    @Override
    public double getAppliedVoltage_V() {
        return sim_getWindingVoltage();
    }

    @Override
    public void resetDistance() {
        curPos_rad = 0;
    }


}
