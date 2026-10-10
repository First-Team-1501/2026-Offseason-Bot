// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.ClimberConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.state.RobotStates;

public class Climber extends PositionSubsystem implements SupurdueperSubsystem {
    /** Creates a new Climber. */
    PositionVoltage noMotionMagic = new PositionVoltage(Rotations.of(0)).withEnableFOC(false);

    public Climber() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Climber/Position", getPosition().in(Rotations));
        DogLog.log("Climber/Setpoint", getSetpoint().in(Rotations));
        if (getCurrentCommand() != null && getCurrentCommand().getName() != null)
            DogLog.log("Climber/Command", getCurrentCommand().getName());
    }

    public Command releaseIntake() {
        return goToPosition(() -> Rotations.of(ClimberConstants.kDropIntakePosition))
                .withName("Release Intake");
    }

    public Command climb() {
        return goToPosition(() -> Rotations.of(ClimberConstants.kClimbPosition)).withName("Climb");
    }

    public Command home() {
        return goToPosition(() -> Rotations.of(0)).withName("Home");
    }

    public Command zero() {
        return run(() -> motor.setControl(voltageRequest.withOutput(-3).withIgnoreSoftwareLimits(true)))
                .until(() -> motor.getTorqueCurrent().getValueAsDouble() < -65.0)
                .andThen(runOnce(
                        () -> motor.setControl(voltageRequest.withOutput(0).withIgnoreSoftwareLimits(false))))
                .andThen(runOnce(() -> motor.setPosition(0)));
    }

    @Override
    public Command goToPosition(Supplier<Angle> rotations) {
        return run(() -> motor.setControl(noMotionMagic.withPosition(rotations.get())));
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
        return ClimberConstants.kCurrentLimit;
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
        RobotStates.actionClimb.onTrue(climb());
        RobotStates.actionClimberUp.onTrue(releaseIntake());
        RobotStates.actionClimberHome.onTrue(home());
        RobotStates.auto_drop_intake.onTrue(releaseIntake());
        // RobotStates.testController.leftStickY.whileTrue(
        //       runEnd(() -> runVoltage(Volts.of(12 * RobotStates.testController.getDriveFwdPositive())), this::stop));
        RobotStates.testController.start.onTrue(runOnce(() -> motor.setPosition(0)));
        //  RobotStates.testController.downDpad.onTrue(zero());
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(ClimberConstants.GravityType)
                .withKP(ClimberConstants.kp)
                .withKI(ClimberConstants.ki)
                .withKD(ClimberConstants.kd)
                .withKS(ClimberConstants.ks)
                .withKV(ClimberConstants.kv)
                .withKA(ClimberConstants.ka)
                .withKG(ClimberConstants.kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kV(ClimberConstants.profileKv)
                .withMotionMagicExpo_kA(ClimberConstants.profileKa)
                .withMotionMagicCruiseVelocity(ClimberConstants.profileV)
                .withMotionMagicAcceleration(ClimberConstants.profileA);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(ClimberConstants.kForwardSoftLimit)
                .withReverseSoftLimitThreshold(ClimberConstants.kReverseSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return Rotations.of(ClimberConstants.positionTolerance);
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }
}
