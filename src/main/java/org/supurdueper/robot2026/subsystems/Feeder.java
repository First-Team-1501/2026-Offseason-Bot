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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.FeederConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Feeder extends VelocitySubsystem implements SupurdueperSubsystem {

    /** Creates a new Feeder. */
    public enum FeedState {
        feed(FeederConstants.feedVelocity),
        idle(FeederConstants.idleVelocity),
        purge(FeederConstants.purgeVelocity),
        stop(RPM.of(0));

        public AngularVelocity velocity;

        FeedState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }

    @Getter
    private FeedState feedState;

    public Feeder() {
        config.TorqueCurrent.PeakForwardTorqueCurrent = FeederConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakForwardDutyCycle = 0.0;
        configureMotors();
        Robot.add(this);
        feedState = FeedState.stop;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (feedState.equals(FeedState.stop)) {
            run(() -> stop());
        } else {
            setVelocity(feedState.velocity);
        }
        DogLog.log("Feeder/RPM", getVelocity().in(RPM));
        DogLog.log("Feeder/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Feeder/State", feedState.name());
        DogLog.log("Feeder/At Velocity", isAtVelocityTrigger().getAsBoolean());
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    @Override
    public void bindCommands() {
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.auto_shoot)
                .onTrue(setState(FeedState.feed));
        RobotStates.actionAim
                .or(RobotStates.auto_shoot)
                .or(RobotStates.actionShoot)
                .onFalse(setState(FeedState.idle));
    }

    public Command setState(FeedState velocity) {
        return runOnce(() -> feedState = velocity);
    }

    public boolean atFeed() {
        return feedState.equals(FeedState.feed);
    }

    public boolean atPurge() {
        return feedState.equals(FeedState.purge);
    }

    public boolean atIdle() {
        return feedState.equals(FeedState.idle);
    }

    public boolean atStop() {
        return feedState.equals(FeedState.stop);
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
        return false;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }
}
