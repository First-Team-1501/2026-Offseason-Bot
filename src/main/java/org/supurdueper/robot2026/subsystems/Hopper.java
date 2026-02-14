// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Hopper extends VelocitySubsystem implements SupurdueperSubsystem {
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

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(Constants.HopperConstants.kP)
                .withKI(Constants.HopperConstants.kI)
                .withKD(Constants.HopperConstants.kD)
                .withKS(Constants.HopperConstants.kS)
                .withKV(Constants.HopperConstants.kV)
                .withKA(Constants.HopperConstants.kA);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kA(Constants.HopperConstants.profilekA)
                .withMotionMagicExpo_kV(Constants.HopperConstants.profilekV);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return Constants.HopperConstants.softLimitConfig;
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return Constants.HopperConstants.velocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }
}
