// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class ShooterHood extends PositionSubsystem implements SupurdueperSubsystem {
    private final CANcoder hoodCancoder;

    /** Creates a new ShooterHood. */
    public ShooterHood() {
        hoodCancoder = new CANcoder(CanId.CANCODER_HOOD.getDeviceNumber(), CanId.CANCODER_HOOD.getBus());
        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
                .withMagnetOffset(Constants.ShooerHoodConstants.kAbsEncoderOffset)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        hoodCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(cancoderConfig));

        config = config.withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(CanId.CANCODER_HOOD.getDeviceNumber())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(Constants.ShooerHoodConstants.kAbsEncoderRatio));
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void periodic() {
        super.periodic();
        // This method will be called once per scheduler run
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
                .withMotionMagicExpo_kA(Constants.ShooerHoodConstants.profileKa);
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
