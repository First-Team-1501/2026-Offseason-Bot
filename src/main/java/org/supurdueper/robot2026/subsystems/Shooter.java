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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

public class Shooter extends VelocitySubsystem implements SupurdueperSubsystem {
    /** Creates a new VelocityTest. */
    private VelocityTorqueCurrentFOC currentRequest = new VelocityTorqueCurrentFOC(0);

    private MotionMagicVelocityTorqueCurrentFOC velocityCurrentRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    public enum shooterrps {
        idle,
        stop,
        purge,
        shootFar,
        shootClose,
        feed;
    }

    @Getter
    private shooterrps rpsState;

    public Shooter() {
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_THREE, motor, false);
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FOUR, motor, false);
        followerMotor = TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FIVE, motor, false);

        configureMotors();
        Robot.add(this);

        rpsState = shooterrps.stop;
    }

    public boolean atSpeed() {
        return atVelocity() & rpsState.equals(shooterrps.shootClose);
    }

    public Command setVelocityState(shooterrps velocity) {
        return Commands.runOnce(() -> rpsState = velocity);
    }

    public AngularVelocity getRpsSetpoint(shooterrps rps) {
        AngularVelocity setpoint;
        switch (rps) {
            case shootClose:
                setpoint = Constants.ShooterConstants.kShootCloseRps;
                break;
            case shootFar:
                setpoint = Constants.ShooterConstants.kShootFarRps;
                break;
            case purge:
                setpoint = Constants.ShooterConstants.kPurgeRPS;
            case idle:
                setpoint = Constants.ShooterConstants.kIdleRps;
            case stop:
            default:
                setpoint = Constants.ShooterConstants.kStopRPS;
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

        DogLog.log("Shooter/RPS", getVelocity().in(RotationsPerSecond));
        DogLog.log("Shooter/Target RPS", getRpsSetpoint(rpsState));
        DogLog.log("Shooter/RPS State", rpsState.toString());
        DogLog.log("Shooter/ At Velocity", atVelocity());

        super.periodic();
    }

    @Override
    public CanId canIdLeader() {
        return CanId.SHOOTER_ONE;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.SHOOTER_FOUR;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.ShooterConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return false;
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withKP(Constants.ShooterConstants.kP)
                .withKI(Constants.ShooterConstants.kI)
                .withKD(Constants.ShooterConstants.kD)
                .withKS(Constants.ShooterConstants.kS)
                .withKV(Constants.ShooterConstants.kV);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kA(Constants.ShooterConstants.profilekA)
                .withMotionMagicExpo_kV(Constants.ShooterConstants.profilekV);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return Constants.ShooterConstants.kSoftLimits;
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return Constants.ShooterConstants.kVelocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sysIdConfig'");
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }
}
