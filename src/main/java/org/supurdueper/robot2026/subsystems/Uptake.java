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
import org.supurdueper.robot2026.Constants.UptakeConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Uptake extends VelocitySubsystem implements SupurdueperSubsystem {

    /** Creates a new Uptake. */
    public enum FeedState {
        feed(UptakeConstants.feedVelocity),
        purge(UptakeConstants.purgeVelocity),
        stop(RPM.of(0));

        public AngularVelocity velocity;

        FeedState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }

    @Getter
    private FeedState feedState;

    public Uptake() {
        config.TorqueCurrent.PeakForwardTorqueCurrent = UptakeConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -1 * UptakeConstants.kMaxAmps;
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
        DogLog.log("Uptake/RPM", getVelocity().in(RPM));
        DogLog.log("Uptake/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Uptake/State", feedState.name());
        DogLog.log("Uptake/At Velocity", isAtVelocityTrigger().getAsBoolean());
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    @Override
    public void bindCommands() {
                RobotStates.actionShoot
                .or(RobotStates.auto_shoot)
                .onTrue(setState(FeedState.feed));
        // RobotStates.testController.B.onTrue(goToVelocity(() -> RPM.of(500)));
        // RobotStates.testController.X.onTrue(goToVelocity(() -> RPM.of(1000)));
        // RobotStates.testController.Y.onTrue(goToVelocity(() -> RPM.of(2000)));
        // RobotStates.testController.A.onTrue(run(this::stop));
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

    public boolean atStop() {
        return feedState.equals(FeedState.stop);
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
        return true;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return UptakeConstants.kCurrentLimit;
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
