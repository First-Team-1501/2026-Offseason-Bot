// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import org.supurdueper.robot2026.state.RobotStates;

public class Feeder extends VelocitySubsystem implements SupurdueperSubsystem {

    private VelocityTorqueCurrentFOC currentRequest = new VelocityTorqueCurrentFOC(0);
    private MotionMagicVelocityTorqueCurrentFOC velocityCurrentRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    /** Creates a new Feeder. */
    public enum feedRPS {
        feed,
        purge,
        stop;
    }

    @Getter
    private feedRPS rpsState;

    public Feeder() {
        configureMotors();
        Robot.add(this);

        rpsState = feedRPS.stop;
    }

    public AngularVelocity getRpsSetpoint(feedRPS rps) {
        AngularVelocity setpoint;
        switch (rps) {
            case feed:
                setpoint = Constants.FeederConstants.kFeedRPS;
                break;
            case purge:
                setpoint = Constants.FeederConstants.kPurgeRPS;
                break;
            case stop:
            default:
                setpoint = Constants.FeederConstants.kStopRPS;
        }
        return setpoint;
    }

    @Override
    protected void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityCurrentRequest.withVelocity(velocity));
    }

    @Override
    protected void setVelocity(double velocity) {
        motor.setControl(velocityCurrentRequest.withVelocity(velocity));
    }

    public Command goToVelocity() {
        return goToVelocity(() -> getRpsSetpoint(rpsState));
    }

    public Command goToVelocityBlocking() {
        return goToVelocity(() -> getRpsSetpoint(rpsState)).withName("goToRPS(" + rpsState.toString() + ")");
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Feeder/RPS", getVelocity().in(RotationsPerSecond));
        DogLog.log("Feeder/Target RPS", getRpsSetpoint(rpsState));
        DogLog.log("Feeder/RPS State", rpsState.toString());
        DogLog.log("Feeder/ At Velocity", atVelocity());
    }

    @Override
    public void bindCommands() {
        RobotStates.actionShoot.and(RobotStates.shooterAtSpeed).onTrue(setRPSState(rpsState.feed));
    }

    public Command setRPSState(feedRPS velocity) {
        return runOnce(() -> rpsState = velocity);
    }

    public boolean atFeed() {
        return rpsState.equals(feedRPS.feed);
    }

    public boolean atPurge() {
        return rpsState.equals(feedRPS.purge);
    }

    public boolean atStop() {
        return rpsState.equals(feedRPS.stop);
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(Constants.FeederConstants.kP)
                .withKI(Constants.FeederConstants.kI)
                .withKD(Constants.FeederConstants.kD)
                .withKS(Constants.FeederConstants.kS)
                .withKV(Constants.FeederConstants.kV)
                .withKA(Constants.FeederConstants.kA);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kA(Constants.FeederConstants.profilekA)
                .withMotionMagicExpo_kV(Constants.FeederConstants.profilekV);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return Constants.FeederConstants.softLimitConfig;
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
        return true;
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
