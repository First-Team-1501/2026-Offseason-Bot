// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.RPM;

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
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Feeder extends VelocitySubsystem implements SupurdueperSubsystem {

    /** Creates a new Feeder. */
    public enum FeedState {
        feed(Constants.FeederConstants.kFeedRPS),
        purge(Constants.FeederConstants.kPurgeRPS),
        stop(RPM.of(0));

        public AngularVelocity velocity;

        FeedState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }

    @Getter
    private FeedState feedState;

    public Feeder() {
        configureMotors();
        Robot.add(this);

        feedState = FeedState.stop;
    }

    @Override
    public void periodic() {
        super.periodic();
        setVelocity(feedState.velocity);
        DogLog.log("Feeder/RPM", getVelocity().in(RPM));
        DogLog.log("Feeder/Target RPM", feedState.velocity.in(RPM));
        DogLog.log("Feeder/State", feedState.name());
        DogLog.log("Feeder/At Velocity", atVelocity());
    }

    @Override
    public void bindCommands() {}

    public Command setRPSState(FeedState velocity) {
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
                .withKP(Constants.FeederConstants.kP)
                .withKI(0)
                .withKD(0)
                .withKS(Constants.FeederConstants.kS)
                .withKV(Constants.FeederConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return Constants.FeederConstants.velocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
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
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.FeederConstants.kCurrentLimit;
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
