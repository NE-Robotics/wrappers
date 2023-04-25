package com.northeasternrobotics.wrappers.motorcontrollers.PlayingWithFusion;

import com.northeasternrobotics.wrappers.HardwareWrapper;
import com.northeasternrobotics.wrappers.motorcontrollers.AbstractSimmableMotorController;
import com.northeasternrobotics.wrappers.motorcontrollers.CTRE.RealTalonFX;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.math.util.Units;

public class RealVenom extends AbstractSimmableMotorController {
    CANVenom _venom;
    public RealVenom(int can_id) {
        _venom = new CANVenom(can_id);
    }

    @Override
    public void setInverted(boolean invert) {
        _venom.setInverted(invert);
    }

    @Override
    public void setNeutralMode(NeutralMode mode) {
        switch(mode) {
            case UseSavedMode:
            case Coast:
                _venom.setBrakeCoastMode(CANVenom.BrakeCoastMode.Coast);
                break;
            case Brake:
                _venom.setBrakeCoastMode(CANVenom.BrakeCoastMode.Brake);
                break;
        }
    }

    @Override
    public void setClosedLoopGains(double p, double i, double d) {
        _venom.setKP(p);
        _venom.setKI(i);
        _venom.setKD(d);
    }

    @Override
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        _venom.setKF(arbFF_V);
        // TODO check if this is the correct conversion
        _venom.setCommand(CANVenom.ControlMode.SpeedControl, Units.radiansPerSecondToRotationsPerMinute(velocityCmd_radpersec));
    }

    @Override
    public void setVoltageCmd(double cmd_v) {
        var pctCmd = cmd_v / HardwareWrapper.k_maxBatteryVoltage;

        if (pctCmd > 1.0) {
            pctCmd = 1.0;
        }

        if (pctCmd < -1.0) {
            pctCmd = -1.0;
        }

        _venom.setCommand(CANVenom.ControlMode.Proportional, pctCmd);
    }

    @Override
    public void overrideDefaultNativeUnitsPerRotation(double nativeUnitsPerRotation) {
        System.out.println("Venom: overrideDefaultNativeUnitsPerRotation not implemented");
    }

    @Override
    public double getCurrent_A() {
        return _venom.getOutputCurrent();
    }

    @Override
    public double getVelocity_radpersec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(_venom.getSpeed());
    }

    @Override
    public void follow(Object leader) {
        if (leader.getClass() == RealVenom.class) {
            _venom.follow(((RealVenom) leader)._venom);
        } else {
            throw new IllegalArgumentException(leader.getClass().toString() + " cannot be followed by a " + RealTalonFX.class.toString());
        }
    }

    @Override
    public double getPosition_rad() {
        return Units.rotationsToRadians(_venom.getPosition());
    }

    @Override
    public double getAppliedVoltage_V() {
        return _venom.getOutputVoltage();
    }

    @Override
    public void resetDistance() {
        _venom.setPosition(0);
    }
}
