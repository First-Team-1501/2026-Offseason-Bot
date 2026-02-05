// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Volts;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Intake extends TalonFXSubsystem implements SupurdueperSubsystem {
    /** Creates a new Intake. */
    public Intake() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return CanId.INTAKE_ONE;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.INTAKE_TWO;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.IntakeConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    public void run() {
        runVoltage(Constants.IntakeConstants.kIntakeVoltage);
    }

    public void purge() {
        runVoltage(Constants.IntakeConstants.kPurgeVoltage);
    }

    public Command runintake() {
        return Commands.runEnd(this::run, this::stop);
    }

    public Command runPurge() {
        return Commands.runEnd(this::purge, this::stop);
    }


    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }


}
