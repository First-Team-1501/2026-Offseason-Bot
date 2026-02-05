// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Hopper extends TalonFXSubsystem implements SupurdueperSubsystem {
    /** Creates a new Hopper. */
    public Hopper() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return CanId.INDEXER_ONE;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.INDEXER_TWO;
    }

    @Override
    public boolean followerInverted() {
       return true;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
       return Constants.HopperConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
       return false;
    }

    public void run() {
        runVoltage(Constants.HopperConstants.kIntakeVoltage);
    }

    public void purge() {
        runVoltage(Constants.HopperConstants.kPurgeVoltage);
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }
}
