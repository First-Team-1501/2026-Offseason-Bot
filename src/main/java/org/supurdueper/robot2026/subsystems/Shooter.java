// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Shooter extends VelocitySubsystem implements SupurdueperSubsystem {
    /** Creates a new VelocityTest. */
    private VelocityTorqueCurrentFOC currentRequest = new VelocityTorqueCurrentFOC(0);

    private MotionMagicVelocityTorqueCurrentFOC velocityCurrentRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    public Shooter() {
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_THREE, motor, false);
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FOUR, motor, false);
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FIVE, motor, false);

        configureMotors();
        Robot.add(this);
    }

    public boolean atSpeed() {
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return CanId.SHOOTER_ONE;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.SHOOTER_FOUR;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.ShooterConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(Constants.ShooterConstants.kP)
                .withKI(Constants.ShooterConstants.kI)
                .withKD(Constants.ShooterConstants.kD)
                .withKS(Constants.ShooterConstants.kS)
                .withKV(Constants.ShooterConstants.kV);
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
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }
}
