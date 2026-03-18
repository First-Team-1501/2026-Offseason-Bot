// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.HopperConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Hopper extends VelocitySubsystem implements SupurdueperSubsystem {
    /** Creates a new Hopper. */
    public Hopper() {
        config.TorqueCurrent.PeakForwardTorqueCurrent = HopperConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -1 * HopperConstants.kMaxAmps;
        config = config.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HopperConstants.gearRatio));
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Hopper/Current RPM", getVelocity().in(RPM));
        DogLog.log("Hopper/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Hopper/At Velocity", isAtVelocity());
        DogLog.log("Hopper/ReadyToShoot", RobotStates.infoReadyToShoot.getAsBoolean());
    }

    @Override
    public CanId canIdLeader() {
        return CanId.INDEXER;
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
        return HopperConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }

    public void run() {
        setVelocity(HopperConstants.kIntakeSpeed);
    }

    public void agitate() {
        runVoltage(Volts.of(-2));
    }

    public void purge() {
        runVoltage(HopperConstants.kPurgeVoltage);
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    @Override
    public void bindCommands() {
        RobotStates.actionShoot.whileTrue(runEnd(this::run, this::stop));
        RobotStates.auto_shoot.whileTrue(runEnd(this::run, this::stop));
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(HopperConstants.kP)
                .withKI(0)
                .withKD(0)
                .withKS(HopperConstants.kS)
                .withKV(HopperConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return HopperConstants.velocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }
}
