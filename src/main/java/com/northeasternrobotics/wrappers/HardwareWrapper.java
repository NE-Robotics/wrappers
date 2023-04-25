package com.northeasternrobotics.wrappers;

import edu.wpi.first.wpilibj.PowerDistribution;

public class HardwareWrapper {
    public static PowerDistribution.ModuleType k_pdbModuleType = PowerDistribution.ModuleType.kCTRE;
    public static double k_hardwareSimLoopSeconds = 0.001;
    public static double k_periodicLoopSeconds = 0.02;

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
}
