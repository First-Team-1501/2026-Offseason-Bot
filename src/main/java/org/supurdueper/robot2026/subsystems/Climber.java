// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2026.CanId;

public class Climber extends TalonFXSubsystem {
    /** Creates a new Climber. */
    public Climber() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
