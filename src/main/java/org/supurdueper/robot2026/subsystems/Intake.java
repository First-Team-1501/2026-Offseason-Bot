// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.RPM;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Constants.IntakeConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Intake extends VelocitySubsystem implements SupurdueperSubsystem {
    /** Creates a new Intake. */
    public Intake() {
        config = config.withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kGearRatio));
        configureMotors();
        Robot.add(this);
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
        return true;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.IntakeConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }

    public void intake() {
        runVoltage(Constants.IntakeConstants.kIntakeVoltage);
    }

    public void shoot() {
        runVoltage(Constants.IntakeConstants.kShootVoltage);
    }

    public void purge() {
        runVoltage(Constants.IntakeConstants.kPurgeVoltage);
    }

    public void test() {
        runVoltage(Units.Volts.of(2));
    }

    public Command runIntake() {
        return runEnd(this::intake, this::stop).withName("Intake/runIntake");
    }

    public Command runShoot() {
        return runEnd(this::shoot, this::stop).withName("Intake/runShoot");
    }

    public Command runPurge() {
        return runEnd(this::purge, this::stop).withName("Intake/runPurge");
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Intake/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Intake/Current RPM", getVelocity().in(RPM));
    }

    @Override
    public void bindCommands() {
        RobotStates.actionIntake.whileTrue(runEnd(this::intake, this::stop));
        RobotStates.actionShoot.whileTrue(runEnd(this::intake, this::stop));
        RobotStates.testController.A.onTrue(run(this::stop));
        RobotStates.testController.X.onTrue(goToVelocity(() -> RPM.of(1000)));
        RobotStates.testController.B.onTrue(goToVelocity(() -> RPM.of(2000)));
        RobotStates.testController.Y.onTrue(goToVelocity(() -> RPM.of(4000)));


    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(IntakeConstants.kp)
                .withKI(0)
                .withKD(0)
                .withKS(IntakeConstants.ks)
                .withKV(IntakeConstants.kv)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return IntakeConstants.kVelocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }
}
