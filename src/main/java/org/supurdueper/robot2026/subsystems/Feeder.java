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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.FeederConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;

public class Feeder extends VelocitySubsystem implements SupurdueperSubsystem {

    /** Creates a new Feeder. */
    public Feeder() {
        config.TorqueCurrent.PeakForwardTorqueCurrent = FeederConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        config.Feedback.SensorToMechanismRatio = FeederConstants.kGearRatio;
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Feeder/RPM", getVelocity().in(RPM));
        DogLog.log("Feeder/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Feeder/At Velocity", isAtVelocityTrigger().getAsBoolean());
        DogLog.log("Feeder/Current", motor.getTorqueCurrent().getValue().in(Amps));
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    /** Feeds at the shooter setpoint until interrupted, then stops. */
    public Command feed() {
        return runEnd(() -> setVelocity(RobotContainer.getShooter().getSetpoint()), this::stop)
                .withName("Feeder/feed");
    }

    @Override
    public void bindCommands() {
        // Only while shooting; stopped otherwise, including while aiming or intaking (team, 2026-10-21)
        RobotStates.actionShoot
                .or(RobotStates.auto_shoot)
                .or(RobotStates.actionSetShot)
                .whileTrue(feed());
        RobotStates.testController.B.onTrue(goToVelocity(() -> RPM.of(500)));
        RobotStates.testController.X.onTrue(goToVelocity(() -> RPM.of(1000)));
        RobotStates.testController.Y.onTrue(goToVelocity(() -> RPM.of(2000)));
        RobotStates.testController.A.onTrue(run(this::stop));
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(FeederConstants.kP)
                .withKI(0)
                .withKD(0)
                .withKS(FeederConstants.kS)
                .withKV(FeederConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return FeederConstants.velocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }

    @Override
    public CanId canIdLeader() {
        return CanId.FEEDER_ONE;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.FEEDER_TWO;
    }

    @Override
    public boolean followerInverted() {
        return true;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return FeederConstants.kCurrentLimit;
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
