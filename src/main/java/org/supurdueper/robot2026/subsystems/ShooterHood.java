// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class ShooterHood extends PositionSubsystem implements SupurdueperSubsystem {

    private TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC positionCurrentRequest = new MotionMagicTorqueCurrentFOC(0);

    /** Creates a new ShooterHood. */
    public ShooterHood() {

        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        // This method will be called once per scheduler run
    }

    @Override
    public Command goToPosition(Supplier<Angle> rotations) {
        return run(() -> motor.setControl(positionCurrentRequest.withPosition(rotations.get())));
    }

    private Angle degreesToMotorRotations(Angle degrees) {
        return Rotations.of(degrees.in(Degrees) / Constants.ShooerHoodConstants.kDegreesPerRotation);
    }

    private Angle motorRotationsToDegrees(Angle motorRotations) {
        return Degrees.of(motorRotations.in(Rotations) * Constants.ShooerHoodConstants.kDegreesPerRotation);
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(Constants.ShooerHoodConstants.kp)
                .withKI(Constants.ShooerHoodConstants.ki)
                .withKD(Constants.ShooerHoodConstants.kd)
                .withKS(Constants.ShooerHoodConstants.ks)
                .withKV(Constants.ShooerHoodConstants.kv)
                .withKA(Constants.ShooerHoodConstants.ka)
                .withKG(Constants.ShooerHoodConstants.kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Constants.ShooerHoodConstants.profileKv)
                .withMotionMagicExpo_kA(Constants.ShooerHoodConstants.profileKa)
                .withMotionMagicCruiseVelocity(Constants.ShooerHoodConstants.profileV)
                .withMotionMagicAcceleration(Constants.ShooerHoodConstants.profileA);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.ShooerHoodConstants.kForwardSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.ShooerHoodConstants.kReverseSoftLimit)
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return Constants.ShooerHoodConstants.positionTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }

    @Override
    public CanId canIdLeader() {
        return CanId.SHOOTER_HOOD;
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
        return Constants.ShooerHoodConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }
}
