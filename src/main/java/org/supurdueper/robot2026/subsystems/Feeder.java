// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Feeder extends VelocitySubsystem implements SupurdueperSubsystem {
  /** Creates a new Feeder. */
  public Feeder() {}

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
  public Slot0Configs pidGains() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'pidGains'");
  }

  @Override
  public MotionMagicConfigs motionMagicConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'motionMagicConfig'");
  }

  @Override
  public SoftwareLimitSwitchConfigs softLimitConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'softLimitConfig'");
  }

  @Override
  public AngularVelocity velocityTolerance() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'velocityTolerance'");
  }

  @Override
  public SysIdRoutine sysIdConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
  }

  @Override
  public CanId canIdLeader() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'canIdLeader'");
  }

  @Override
  public CanId canIdFollower() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'canIdFollower'");
  }

  @Override
  public boolean followerInverted() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'followerInverted'");
  }

  @Override
  public CurrentLimitsConfigs currentLimits() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'currentLimits'");
  }

  @Override
  public boolean inverted() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'inverted'");
  }

  @Override
  public boolean brakeMode() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'brakeMode'");
  }
}
