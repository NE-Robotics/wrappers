package com.northeasternrobotics.wrappers;

import java.util.HashMap;

/**
 * Implements a CAN device which can be accessed in simulation. Used for wrapping
 * devices which don't directly support WPIlib's HAL layer simulation style.
 * Enforces a unique CAN ID per device - this isn't actually required on a robot,
 * but is a best-practice to guard against CAN ID conflicts.
 */
public class SimDeviceBanks {

    // CAN DEVICES
    private static final HashMap<Integer, Object> CANBank = new HashMap<Integer, Object>();
    //Digital Input Devices (Encoders)
    private static final HashMap<Integer, Object> DIBank = new HashMap<Integer, Object>();
    //SPI Devices (Gyros, etc.)
    //Digital Input Devices (Encoders)
    private static final HashMap<Integer, Object> SPIBank = new HashMap<Integer, Object>();

    /**
     * Adds a device to the CAN bank
     * @param newDevice The device to add
     * @param id The CAN ID of the device
     */
    public static void addCANDevice(Object newDevice, int id) {

        if (CANBank.containsKey(id)) {
            throw new IllegalStateException("CAN ID " + id + " has already been allocated!");
        }

        CANBank.put(id, newDevice);
    }

    /**
     * Returns the device connected to the given CAN ID
     * @param id CAN ID of the device
     * @return The device connected to the given CAN ID
     */
    public static Object getCANDevice(int id) {
        if (!CANBank.containsKey(id)) {
            throw new IllegalStateException("CAN ID " + id + " is not a device on the CAN bus!");
        }
        return CANBank.get(id);
    }

    /**
     * Adds a device to the DI bank
     * @param newDevice The device to add
     * @param port The port of the device
     */
    public static void addDIDevice(Object newDevice, int port) {
        if (DIBank.containsKey(port)) {
            throw new IllegalStateException("Digital Input Port " + port + " has already been allocated!");
        }
        DIBank.put(port, newDevice);
    }

    /**
     * Returns the device connected to the given digital input port
     * @param port DIO port of the device
     * @return The device connected to the given digital input port
     */
    public static Object getDIDevice(int port) {
        if (!DIBank.containsKey(port)) {
            throw new IllegalStateException("Digital Input Port " + port + " is not a connected device!");
        }
        return DIBank.get(port);
    }

    /**
     * Adds a device to the SPI bank
     * @param newDevice The device to add
     * @param cs The chip select port of the device
     */
    public static void addSPIDevice(Object newDevice, int cs) {
        if (SPIBank.containsKey(cs)) {
            throw new IllegalStateException("SPI cs " + cs + " has already been allocated!");
        }
        SPIBank.put(cs, newDevice);
    }

    /**
     * Returns the device connected to the given SPI chip select port
     * @param cs Slot of the device
     * @return The device connected to the given SPI chip select port
     */
    public static Object getSPIDevice(int cs) {
        if (!SPIBank.containsKey(cs)) {
            throw new IllegalStateException("SPI cs " + cs + " is not a connected device!");
        }
        return SPIBank.get(cs);
    }

    /** Clears all devices from the simulation */
    public static void clearAllBanks() {
        DIBank.clear();
        SPIBank.clear();
        CANBank.clear();
    }
}
