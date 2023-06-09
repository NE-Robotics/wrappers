package com.northeasternrobotics.wrappers;

import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Wrapper class for hardware contains things like PDB type, hardware simulation speed, and periodic loop speed
 */
public class HardwareWrapper {
    private static final double k_safetyMargin = 0.8;
    /**
     * The type of Power Distribution Module your robot uses, default is CTRE
     */
    public static PowerDistribution.ModuleType k_pdbModuleType = PowerDistribution.ModuleType.kCTRE;

    /**
     * The speed at which to run hardware sim loop in seconds, default is 0.001 = 1000 times per second
     */
    public static double k_hardwareSimLoopSeconds = 0.001;

    /**
     * The speed at which to run periodic loop in seconds, default is 0.02 = 50 times per second
     */
    public static double k_periodicLoopSeconds = 0.02;

    /**
     * The maximum voltage of the battery, default is 14.0
     */
    public static double k_maxBatteryVoltage = 14.0;

    /**
     * @param moduleType, the type of Power Distribution Module your robot uses
     */
    public static void setPDBModuleType(PowerDistribution.ModuleType moduleType) {
        k_pdbModuleType = moduleType;
    }

    /**
     * @param speed, the number of times per second to run hardware sim loop
     */
    public static void setHardwareSimulationSpeed(double speed) {
        k_hardwareSimLoopSeconds = 1.0 / speed;
    }

    /**
     * @param speed, the number of times per second to run periodic loop
     */
    public static void setPeriodicLoopSpeed(double speed) {
        k_periodicLoopSeconds = 1.0 / speed;
    }

    /**
     * Configuring this to your battery increases the accuracy of voltage commands with Talons and Venom motors
     *
     * @param maxVoltage, the maximum voltage of the battery
     */
    public static void setMaximumBatteryVoltage(double maxVoltage) {
        k_maxBatteryVoltage = maxVoltage;
    }

    /**
     * Automatically detects your current batteries maximum voltage for increased accuracy of voltage commands
     * Should be run in robotInit() but after setPDNModuleType()
     */
    public static void autoDetectMaximumBatteryVoltage() {
        var pdp = new PowerDistribution(1, k_pdbModuleType);
        k_maxBatteryVoltage = pdp.getVoltage() + k_safetyMargin;
    }
}
