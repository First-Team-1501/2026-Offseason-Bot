// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Indexer extends TalonFXSubsystem implements SupurdueperSubsystem {
    /** Creates a new Indexer. */
    public Indexer() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
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
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.IndexerConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }

    public void Intake() {
        runVoltage(Constants.IndexerConstants.kIndexShooterVoltage);
    }

    public void Shooter() {
        runVoltage(Constants.IndexerConstants.kIndexShooterVoltage);
    }

    public void Purge() {
        runVoltage(Constants.IndexerConstants.kPurgeVoltage);
    }

    public Command runIndexerIntake() {
        return Commands.runEnd(this::Intake, this::stop);
    }

    public Command runIndexerShooter() {
        return Commands.runEnd(this::Shooter, this::stop);
    }

    public Command runIndexerPurge() {
        return Commands.runEnd(this::Purge, this::stop);
    }
}
