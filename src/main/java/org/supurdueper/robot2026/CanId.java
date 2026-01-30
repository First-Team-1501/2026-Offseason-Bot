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
    CANDLE(29, Constants.rioBus),

    // Intake
    INTAKE(9, Constants.canivoreBus),
    // Indexer
    INDEXER_ONE(10, Constants.canivoreBus),
    INDEXER_TWO(11, Constants.canivoreBus),
    INDEXER_THREE(12, Constants.canivoreBus),

    // Shooter
    SHOOTER_HOOD(13, Constants.canivoreBus),
    CANCODER_HOOD(26, Constants.canivoreBus),
    SHOOTER_ONE(14, Constants.canivoreBus),
    SHOOTER_TWO(15, Constants.canivoreBus),
    SHOOTER_THREE(16, Constants.canivoreBus),
    SHOOTER_FOUR(17, Constants.canivoreBus),
    SHOOTER_FIVE(18, Constants.canivoreBus),
    SHOOTER_SIX(19, Constants.canivoreBus),

    // CLIMBER
    CLIMBER(20, Constants.canivoreBus);

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
