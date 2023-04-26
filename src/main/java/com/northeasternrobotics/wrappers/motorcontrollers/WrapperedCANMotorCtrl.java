package com.northeasternrobotics.wrappers.motorcontrollers;

import com.northeasternrobotics.wrappers.motorcontrollers.CTRE.RealTalonFX;
import com.northeasternrobotics.wrappers.motorcontrollers.CTRE.RealTalonSRX;
import com.northeasternrobotics.wrappers.motorcontrollers.PlayingWithFusion.RealVenom;
import com.northeasternrobotics.wrappers.motorcontrollers.REV.RealSparkMax;
import com.northeasternrobotics.wrappers.motorcontrollers.Sim.SimSmartMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Wrapper for a CAN motor controller
 */
public class WrapperedCANMotorCtrl {

    AbstractSimmableMotorController ctrl;
    private double appliedVoltage;
    private double actVel;
    private double current;
    private double desVel;
    private double actPos;

    /**
     * Constructor for a wrapped CAN motor controller
     *
     * @param prefix Readable name for the motor controller
     * @param can_id CAN ID of the motor controller
     * @param type   Controller type for the real motor controller
     */
    public WrapperedCANMotorCtrl(String prefix, int can_id, CANMotorCtrlType type) {

        System.out.print("=> Starting motor controller init for " + prefix + " CANID = " + can_id);

        if (RobotBase.isSimulation()) {
            ctrl = new SimSmartMotor(can_id);
        } else {
            switch (type) {
                case TALON_FX:
                    ctrl = new RealTalonFX(can_id);
                    break;
                case TALON_SRX:
                    ctrl = new RealTalonSRX(can_id);
                    break;
                case VENOM:
                    ctrl = new RealVenom(can_id);
                case SPARK_MAX:
                    ctrl = new RealSparkMax(can_id);
                    break;
            }
        }
        System.out.println(" ... Done!");

    }

    /**
     * Updates the abstracted controllers values
     */
    public void update() {
        actVel = ctrl.getVelocity_radpersec();
        actPos = ctrl.getPosition_rad();
        current = ctrl.getCurrent_A();
        appliedVoltage = ctrl.getAppliedVoltage_V();
    }

    /**
     * Sets the motor to inverted or not
     *
     * @param invert true if inverted
     */
    public void setInverted(boolean invert) {
        ctrl.setInverted(invert);
    }

    /**
     * Sets the closed loop gain values for P I D
     *
     * @param p proportional term
     * @param i integral term
     * @param d derivative term
     */
    public void setClosedLoopGains(double p, double i, double d) {
        ctrl.setClosedLoopGains(p, i, d);
    }

    /**
     * Sets the closed loop command for the motor in velocity and arbitrary feed forward
     *
     * @param velocityCmd_radpersec desired velocity in radians per second
     * @param arbFF_V               arbitrary feed forward in volts
     */
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        ctrl.setClosedLoopCmd(velocityCmd_radpersec, arbFF_V);
        desVel = velocityCmd_radpersec;
    }

    /**
     * Sets the voltage command for the motor
     *
     * @param cmd_V desired voltage in volts
     */
    public void setVoltageCmd(double cmd_V) {
        ctrl.setVoltageCmd(cmd_V);
    }

    /**
     * @return the current in Amps
     */
    public double getCurrent_A() {
        return current;
    }

    /**
     * @return the actual velocity of the motor in radians per second
     */
    public double getVelocity_radpersec() {
        return actVel;
    }

    /**
     * @return the position in radians
     */
    public double getPosition_rad() {
        return actPos;
    }

    /**
     * Resets the sensor/encoder distance to 0
     */
    public void resetDistance() {
        ctrl.resetDistance();
    }

    /**
     * Motor controller types for real robot
     */
    public enum CANMotorCtrlType {
        /**
         * Talon FX motor controller, ex Falcon 500
         */
        TALON_FX,
        /**
         * Talon SRX motor controller, ex Talon SRX connected to a BAG motor
         */
        TALON_SRX,
        /**
         * Venom motor controller CIM system
         */
        VENOM,
        /**
         * Spark Max motor controller, ex NEO 550
         */
        SPARK_MAX
    }
}
