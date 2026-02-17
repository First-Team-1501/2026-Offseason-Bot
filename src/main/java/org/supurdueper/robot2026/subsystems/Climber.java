// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;

import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.Feeder.FeedState;

public class Climber extends PositionSubsystem implements SupurdueperSubsystem {
    /** Creates a new Climber. */
    public enum ClimbState {
        climbPrep(Constants.ClimberConstants.kPrepClimbPosition),
        dropIntake(Constants.ClimberConstants.kDropIntakePosition),
        home(Inches.of(0));

        public Distance position;

        ClimbState(Distance position) {
            this.position = position;
        }
    }

    @Getter
    private ClimbState climbstate;

    public Climber() {
        configureMotors();
        Robot.add(this);
    }

    public Command setClimbState(ClimbState position) {
        return runOnce(() -> climbstate = position);
    }


    @Override
    public void periodic() {
        super.periodic();
    }


    public Command goToPosition(double rotations) {
        return goToPosition(() -> Rotations.of(rotations));
    }

    @Override
    public CanId canIdLeader() {
        return (CanId.CLIMBER);
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
        return Constants.ClimberConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    @Override
    public void bindCommands() {

    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(Constants.ClimberConstants.GravityType)
                .withKP(Constants.ClimberConstants.kp)
                .withKI(Constants.ClimberConstants.ki)
                .withKD(Constants.ClimberConstants.kd)
                .withKS(Constants.ClimberConstants.ks)
                .withKV(Constants.ClimberConstants.kv)
                .withKA(Constants.ClimberConstants.ka)
                .withKG(Constants.ClimberConstants.kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Constants.ClimberConstants.profileKv)
                .withMotionMagicExpo_kA(Constants.ClimberConstants.profileKa)
                .withMotionMagicCruiseVelocity(Constants.ClimberConstants.profileV)
                .withMotionMagicAcceleration(Constants.ClimberConstants.profileA);
    }

    private Distance motorRotationToHeight(Angle motorRotations) {
        return Inches.of(motorRotations.in(Rotation) * Constants.ClimberConstants.kInchesPerRotation);
    }

    private Angle heightToMotorRotations(Distance height) {
        return Rotations.of(height.in(Inches) / Constants.ClimberConstants.kInchesPerRotation);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(heightToMotorRotations(Constants.ClimberConstants.kForwardSoftLimit))
                .withReverseSoftLimitThreshold(heightToMotorRotations(Constants.ClimberConstants.kReverseSoftLimit));
    }

    @Override
    public Angle positionTolerance() {
        return heightToMotorRotations(Constants.ClimberConstants.positionTolerance);
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }
}
