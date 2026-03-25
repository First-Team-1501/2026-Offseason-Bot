package org.supurdueper.robot2026;

import com.ctre.phoenix6.CANBus;

public enum CanId {

    // Drive
    TALONFX_DRIVE_FL(1, Constants.canivoreBus),
    TALONFX_STEER_FL(2, Constants.canivoreBus),
    TALONFX_DRIVE_FR(3, Constants.canivoreBus),
    TALONFX_STEER_FR(4, Constants.canivoreBus),
    TALONFX_DRIVE_BL(5, Constants.canivoreBus),
    TALONFX_STEER_BL(6, Constants.canivoreBus),
    TALONFX_DRIVE_BR(7, Constants.canivoreBus),
    TALONFX_STEER_BR(8, Constants.canivoreBus),
    CANCODER_STEER_FL(21, Constants.canivoreBus),
    CANCODER_STEER_FR(22, Constants.canivoreBus),
    CANCODER_STEER_BL(23, Constants.canivoreBus),
    CANCODER_STEER_BR(24, Constants.canivoreBus),
    PIGEON(25, Constants.canivoreBus),
    CANDLE(29, Constants.canivoreBus),

    // Intake
    INTAKE_ONE(9, Constants.canivoreBus),
    INTAKE_TWO(10, Constants.canivoreBus),

    // Hopper
    INDEXER(12, Constants.canivoreBus),
    UPTAKE(11, Constants.canivoreBus),

    // Feeder
    FEEDER_ONE(13, Constants.canivoreBus),
    FEEDER_TWO(14, Constants.canivoreBus),

    // Shooter Hood
    SHOOTER_HOOD(15, Constants.canivoreBus),
    CANCODER_HOOD(26, Constants.canivoreBus),

    // Shooter
    SHOOTER_ONE(16, Constants.canivoreBus),
    SHOOTER_TWO(17, Constants.canivoreBus),
    SHOOTER_THREE(18, Constants.canivoreBus),
    SHOOTER_FOUR(19, Constants.canivoreBus),
    SHOOTER_FIVE(20, Constants.canivoreBus),

    // CLIMBER
    CLIMBER(21, Constants.canivoreBus);

    private final int mDeviceNumber;
    private final CANBus mBus;

    CanId(int mDeviceNumber, CANBus mBus) {
        this.mDeviceNumber = mDeviceNumber;
        this.mBus = mBus;
    }

    public int getDeviceNumber() {
        return mDeviceNumber;
    }

    public CANBus getBus() {
        return mBus;
    }
}
