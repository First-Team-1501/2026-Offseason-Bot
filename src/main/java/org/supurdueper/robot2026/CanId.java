package org.supurdueper.robot2026;

import com.ctre.phoenix6.CANBus;

public enum CanId {

    // Drive
    TALONFX_DRIVE_FL(1, Constants.lowerBus),
    TALONFX_STEER_FL(2, Constants.lowerBus),
    TALONFX_DRIVE_FR(7, Constants.lowerBus),
    TALONFX_STEER_FR(8, Constants.lowerBus),
    TALONFX_DRIVE_BL(4, Constants.lowerBus),
    TALONFX_STEER_BL(5, Constants.lowerBus),
    TALONFX_DRIVE_BR(10, Constants.lowerBus),
    TALONFX_STEER_BR(11, Constants.lowerBus),
    CANCODER_STEER_FL(3, Constants.lowerBus),
    CANCODER_STEER_FR(9, Constants.lowerBus),
    CANCODER_STEER_BL(6, Constants.lowerBus),
    CANCODER_STEER_BR(12, Constants.lowerBus),
    PIGEON(15, Constants.lowerBus),
    CANDLE(29, Constants.upperBus),

    // Intake
    INTAKE_ONE(6, Constants.upperBus),
    INTAKE_TWO(7, Constants.upperBus),

    // Hopper
    INDEXER(5, Constants.upperBus),
    UPTAKE(8, Constants.upperBus),

    // Feeder
    FEEDER_ONE(3, Constants.upperBus),
    FEEDER_TWO(10, Constants.upperBus),

    // Shooter Hood
    SHOOTER_HOOD(9, Constants.upperBus),

    // Shooter
    SHOOTER_ONE(1, Constants.upperBus),
    SHOOTER_TWO(2, Constants.upperBus),
    SHOOTER_THREE(4, Constants.upperBus),
    SHOOTER_FOUR(11, Constants.upperBus),
    SHOOTER_FIVE(12, Constants.upperBus),

    // CLIMBER
    CLIMBER(21, Constants.upperBus);

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
