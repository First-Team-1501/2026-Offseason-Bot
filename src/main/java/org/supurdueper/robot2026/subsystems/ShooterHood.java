// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static org.supurdueper.robot2026.Constants.ShooerHoodConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class ShooterHood extends PositionSubsystem implements SupurdueperSubsystem {

    private PositionVoltage noMagicMotion = new PositionVoltage(0);

    /** Creates a new ShooterHood. */
    public ShooterHood() {
        config = config.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio))
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(kPeakForwardVoltage)
                        .withPeakReverseVoltage(kPeakReverseVoltage));
        configureMotors();
        Robot.add(this);
        motor.setPosition(kZeroPosition);
    }

    @Override
    protected void setPosition(Angle position) {
        motor.setControl(noMagicMotion.withPosition(position));
    }

    @Override
    protected void setPosition(double position) {
        motor.setControl(noMagicMotion.withPosition(position));
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("ShooterHood/Position (Deg)", getPosition().in(Degrees));
        DogLog.log("ShooterHood/Target Position (Deg)", getSetpoint().in(Degrees));
        if ((getCurrentCommand() != null) && (getCurrentCommand().getName() != null)) {
            DogLog.log("ShooterHood/Command", getCurrentCommand().getName());
        }
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
        return null;
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
        // RobotStates.actionTestA.onTrue(run(() -> stop()).withName("stop"));
        // RobotStates.actionTestB.onTrue(goToPosition(() -> Degrees.of(22)).withName("22"));
        // RobotStates.actionTestX.onTrue(goToPosition(() -> Degrees.of(28)).withName("28"));
        // RobotStates.actionTestY.onTrue(goToPosition(() -> Degrees.of(35)).withName("35"));
    }
}
