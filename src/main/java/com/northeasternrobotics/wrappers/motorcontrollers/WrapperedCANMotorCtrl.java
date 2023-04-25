package com.northeasternrobotics.wrappers.motorcontrollers;

import com.northeasternrobotics.wrappers.motorcontrollers.PlayingWithFusion.RealVenom;
import com.northeasternrobotics.wrappers.motorcontrollers.Sim.SimSmartMotor;
import com.northeasternrobotics.wrappers.motorcontrollers.REV.RealSparkMax;
import com.northeasternrobotics.wrappers.motorcontrollers.CTRE.RealTalonFX;
import com.northeasternrobotics.wrappers.motorcontrollers.CTRE.RealTalonSRX;
import edu.wpi.first.wpilibj.RobotBase;

public class WrapperedCANMotorCtrl {

    AbstractSimmableMotorController ctrl;
    private double appliedVoltage;
    private double actVel;
    private double current;
    private double desVel;
    private double actPos;

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

    public void update() {
        actVel = ctrl.getVelocity_radpersec();
        actPos = ctrl.getPosition_rad();
        current = ctrl.getCurrent_A();
        appliedVoltage = ctrl.getAppliedVoltage_V();
    }

    public void setInverted(boolean invert) {
        ctrl.setInverted(invert);
    }

    public void setClosedLoopGains(double p, double i, double d) {
        ctrl.setClosedLoopGains(p, i, d);
    }

    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        ctrl.setClosedLoopCmd(velocityCmd_radpersec, arbFF_V);
        desVel = velocityCmd_radpersec;
    }

    public void setVoltageCmd(double cmd_V) {
        ctrl.setVoltageCmd(cmd_V);
    }

    public double getCurrent_A() {
        return current;
    }

    public double getVelocity_radpersec() {
        return actVel;
    }

    public double getPosition_rad() {
        return actPos;
    }

    public void resetDistance() {
        ctrl.resetDistance();
    }

    public enum CANMotorCtrlType {
        TALON_FX,
        TALON_SRX,
        VENOM,
        SPARK_MAX
    }
}
