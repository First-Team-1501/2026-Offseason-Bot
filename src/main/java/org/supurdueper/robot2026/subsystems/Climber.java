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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Climber extends PositionSubsystem implements SupurdueperSubsystem {
    /** Creates a new Climber. */
    private TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

    private MotionMagicTorqueCurrentFOC positionCurrentRequest = new MotionMagicTorqueCurrentFOC(0);

    public Climber() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command releaseIntake() {
        return Commands.runOnce(() ->
                        goToPosition(() -> heightToMotorRotations(Constants.ClimberConstants.kDropIntakePosition)))
                .andThen(() -> goToPosition(() -> heightToMotorRotations(Constants.ClimberConstants.kHomePosition)));
    }

    public Command prepClimb() {
        return Commands.runOnce(
                () -> goToPosition(() -> heightToMotorRotations(Constants.ClimberConstants.kPrepClimbPosition)));
    }

    public Command climb() {
        return Commands.runOnce(
                () -> goToPosition(() -> heightToMotorRotations(Constants.ClimberConstants.kHomePosition)));
    }

    @Override
    public Command goToPosition(Supplier<Angle> rotations) {
        return run(() -> motor.setControl(positionCurrentRequest.withPosition(rotations.get())));
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
        RobotStates.auto.onTrue(releaseIntake());
        RobotStates.actionClimbPrep.onTrue(prepClimb());
        RobotStates.actionClimb.onTrue(climb());
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
