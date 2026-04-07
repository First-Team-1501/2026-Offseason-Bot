// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Constants.LookupTables;
import org.supurdueper.robot2026.Constants.ShooterHoodConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.utils.FieldCalculations;

public class ShooterHood extends PositionSubsystem implements SupurdueperSubsystem {

    private PositionVoltage noMagicMotion = new PositionVoltage(0);
    private final LoggedTunableNumber shooterAngle;

    /** Creates a new ShooterHood. */
    public ShooterHood() {
        config = config.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ShooterHoodConstants.gearRatio))
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(ShooterHoodConstants.kPeakForwardVoltage)
                        .withPeakReverseVoltage(ShooterHoodConstants.kPeakReverseVoltage));
        configureMotors();
        Robot.add(this);
        motor.setPosition(ShooterHoodConstants.kZeroPosition);
        shooterAngle = new LoggedTunableNumber("Shot Tuning/Angle (Deg)");
        shooterAngle.initDefault(22);
    }

    @Override
    protected void setPosition(Angle position) {
        motor.setControl(noMagicMotion.withPosition(position));
    }

    @Override
    protected void setPosition(double position) {
        motor.setControl(noMagicMotion.withPosition(position));
    }

    public Command releaseIntake() {
        return goToPosition(() -> ShooterHoodConstants.kForwardSoftLimit);
    }

    public Command zero() {
        return run(() -> motor.setControl(voltageRequest.withOutput(-3).withIgnoreSoftwareLimits(true)))
                .until(() -> motor.getStatorCurrent().getValueAsDouble() > 20.0)
                .andThen(runOnce(
                        () -> motor.setControl(voltageRequest.withOutput(0).withIgnoreSoftwareLimits(false))))
                .andThen(runOnce(() -> motor.setPosition(ShooterHoodConstants.kZeroPosition)));
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("ShooterHood/Position (Deg)", getPosition().in(Degrees));
        DogLog.log("ShooterHood/Target Position (Deg)", getSetpoint().in(Degrees));
        DogLog.log("ShooterHood/Current", motor.getTorqueCurrent().getValue().in(Amps));
        if (Constants.tuningMode) {
            SmartDashboard.putNumber(
                    "Tuning/Shot Tuning/Distance",
                    FieldCalculations.distanceToGoal(
                                    RobotContainer.getDrivetrain().getState().Pose)
                            .in(Meters));
        }
    }

    public Angle getShotAngle() {
        if (Constants.tuningMode) {
            return Degrees.of(shooterAngle.get());
        } else {
            double distanceToGoalMeters = FieldCalculations.distanceToGoal(
                            RobotContainer.getDrivetrain().getState().Pose)
                    .in(Meters);
            return Degrees.of(LookupTables.distanceToShooterAngle.get(distanceToGoalMeters));
        }
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(ShooterHoodConstants.kp)
                .withKI(ShooterHoodConstants.ki)
                .withKD(ShooterHoodConstants.kd)
                .withKS(ShooterHoodConstants.ks)
                .withKV(ShooterHoodConstants.kv)
                .withKA(ShooterHoodConstants.ka)
                .withKG(ShooterHoodConstants.kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs()
                .withMotionMagicExpo_kV(ShooterHoodConstants.profileKv)
                .withMotionMagicExpo_kA(ShooterHoodConstants.profileKa)
                .withMotionMagicCruiseVelocity(ShooterHoodConstants.profileV)
                .withMotionMagicAcceleration(ShooterHoodConstants.profileA);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(ShooterHoodConstants.kForwardSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterHoodConstants.kReverseSoftLimit)
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return ShooterHoodConstants.positionTolerance;
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
        return ShooterHoodConstants.kCurrentLimit;
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
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.auto_aim)
                .or(RobotStates.auto_shoot)
                .whileTrue(goToPosition(this::getShotAngle));
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.auto_aim)
                .onFalse(goToPosition(() -> Degrees.of(0)));
        RobotStates.auto_drop_intake.onTrue(releaseIntake());
        RobotStates.testController.downDpad.onTrue(zero());
        // RobotStates.testController.leftStickY.whileTrue(
        //        runEnd(() -> runVoltage(Volts.of(12 * RobotStates.testController.getDriveFwdPositive())),
        // this::stop));
        // RobotStates.actionTestA.onTrue(run(() -> stop()).withName("stop"));
        // RobotStates.actionTestB.onTrue(goToPosition(() -> Degrees.of(10)).withName("10"));
        // RobotStates.actionTestX.onTrue(goToPosition(() -> Degrees.of(17)).withName("17"));
        // RobotStates.actionTestY.onTrue(goToPosition(() -> Degrees.of(25)).withName("25"));
    }
}
