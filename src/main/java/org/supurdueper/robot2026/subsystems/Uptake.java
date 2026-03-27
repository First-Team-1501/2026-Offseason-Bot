// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.UptakeConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Uptake extends VelocitySubsystem implements SupurdueperSubsystem {

    /** Creates a new Uptake. */
    public Uptake() {
        config.TorqueCurrent.PeakForwardTorqueCurrent = UptakeConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -1 * UptakeConstants.kMaxAmps;
        config.Feedback.SensorToMechanismRatio = 27 / 14.0;
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Uptake/RPM", getVelocity().in(RPM));
        DogLog.log("Uptake/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Uptake/At Velocity", isAtVelocityTrigger().getAsBoolean());
        DogLog.log("Uptake/Current", motor.getTorqueCurrent().getValue().in(Amps));
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    @Override
    public void bindCommands() {
        RobotStates.actionShoot
                .or(RobotStates.auto_shoot)
                .whileTrue(Commands.waitUntil(RobotStates.infoShooterAtSpeed)
                        .andThen(startEnd(() -> setVelocity(UptakeConstants.feedVelocity), this::stop)));
        ;
        RobotStates.actionIntake
                .and(RobotStates.actionShoot.negate())
                .whileTrue(startEnd(() -> setVoltage(() -> -6), this::stop));
        // RobotStates.testController.B.onTrue(goToVelocity(() -> RPM.of(500)));
        // RobotStates.testController.X.onTrue(goToVelocity(() -> RPM.of(1000)));
        // RobotStates.testController.Y.onTrue(goToVelocity(() -> RPM.of(2000)));
        // RobotStates.testController.A.onTrue(run(this::stop));
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(UptakeConstants.kP)
                .withKI(0)
                .withKD(0)
                .withKS(UptakeConstants.kS)
                .withKV(UptakeConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return UptakeConstants.velocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }

    @Override
    public CanId canIdLeader() {
        return CanId.UPTAKE;
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
        return UptakeConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }
}
