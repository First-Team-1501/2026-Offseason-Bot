// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Shooter extends VelocitySubsystem implements SupurdueperSubsystem {
    /** Creates a new VelocityTest. */
    public Shooter() {
        configureMotors();
        // Manually create followers since we have more than two
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_TWO, motor, false);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_THREE, motor, true);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FOUR, motor, true);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FIVE, motor, true);
        Robot.add(this);
    }

    @Override
    public void periodic() {
        DogLog.log("Shooter/Current RPM", getVelocity().in(RPM));
        DogLog.log("Shooter/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Shooter/At Velocity", atVelocity());
        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return CanId.SHOOTER_ONE;
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
                .withKI(0)
                .withKD(0)
                .withKS(Constants.ShooterConstants.kS)
                .withKV(Constants.ShooterConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return Constants.ShooterConstants.kVelocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    @Override
    public void bindCommands() {
        RobotStates.actionTestShooter.whileTrue(runEnd(this::test, this::stop));
    }

    // Manually creating followers in constructor
    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
