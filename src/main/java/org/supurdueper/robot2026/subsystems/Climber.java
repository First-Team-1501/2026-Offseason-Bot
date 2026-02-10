// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Climber extends PositionSubsystem implements SupurdueperSubsystem {
    /** Creates a new Climber. */
    public Climber() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return (CanId.CLIMBER);
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.ClimberConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(Constants.ClimberConstants.GravityType)
                .withKP(Constants.ClimberConstants.kp)
                .withKI(Constants.ClimberConstants.ki)
                .withKD(Constants.ClimberConstants.kd)
                .withKS(Constants.ClimberConstants.ks)
                .withKV(Constants.ClimberConstants.kv)
                .withKA(Constants.ClimberConstants.ka)
                .withKG(Constants.ClimberConstants.kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Constants.ClimberConstants.profileKv)
                .withMotionMagicExpo_kA(Constants.ClimberConstants.profileKa);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.ClimberConstants.kForwardSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.ClimberConstants.kReverseSoftLimit)
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return Constants.ClimberConstants.positionTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }
}
