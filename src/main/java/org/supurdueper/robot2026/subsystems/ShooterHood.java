// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.ShooterHoodConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.utils.FieldCalculations;
import org.supurdueper.robot2026.utils.ShotCalculator;

public class ShooterHood extends PositionSubsystem implements SupurdueperSubsystem {

    private static final LoggedTunableNumber stowMarginDeg =
            new LoggedTunableNumber("ShooterHood/Stow Margin (Deg)", ShooterHoodConstants.kStowMarginDeg);

    private PositionVoltage noMagicMotion = new PositionVoltage(0).withEnableFOC(false);

    /** Creates a new ShooterHood. */
    public ShooterHood() {
        config = config.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ShooterHoodConstants.gearRatio))
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(ShooterHoodConstants.kPeakForwardVoltage)
                        .withPeakReverseVoltage(ShooterHoodConstants.kPeakReverseVoltage));
        configureMotors();
        telemetry
                .withExtra("hoodDownThresholdDeg", () -> downThreshold().in(Degrees))
                .withExtra("stowAngleDeg", () -> stowAngle().in(Degrees));
        Robot.add(this);
        motor.setPosition(ShooterHoodConstants.kZeroPosition);
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
        return goToPosition(() -> Degrees.of(20));
    }

    /** Drives to the bottom stop (the flywheel interlock cuts the shooter there), re-zeroes, then returns to rest. */
    public Command zero() {
        return run(() -> motor.setControl(voltageRequest.withOutput(-3).withIgnoreSoftwareLimits(true)))
                .until(() -> motor.getStatorCurrent().getValueAsDouble() > 50.0)
                .andThen(runOnce(
                        () -> motor.setControl(voltageRequest.withOutput(0).withIgnoreSoftwareLimits(false))))
                .andThen(runOnce(() -> motor.setPosition(ShooterHoodConstants.kZeroPosition)))
                .andThen(goToPosition(this::restAngle));
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("ShooterHood/Position (Deg)", getPosition().in(Degrees));
        DogLog.log("ShooterHood/Target Position (Deg)", getSetpoint().in(Degrees));
        DogLog.log("ShooterHood/Current", motor.getTorqueCurrent().getValue().in(Amps));
        DogLog.log("ShooterHood/Down", isDown());
        DogLog.log("ShooterHood/In Trench Zone", inTrenchZone());
        DogLog.log("ShooterHood/Rest Angle (Deg)", restAngle().in(Degrees));
    }

    /** Rest angle just above the bottom stop, where the flywheel may run. */
    public static Angle stowAngle() {
        return ShooterHoodConstants.kReverseSoftLimit.plus(Degrees.of(stowMargin()));
    }

    /** Below this measured angle the hood counts as down and the flywheel is unpowered. */
    public static Angle downThreshold() {
        return ShooterHoodConstants.kReverseSoftLimit.plus(Degrees.of(stowMargin() / 2.0));
    }

    private static double stowMargin() {
        return MathUtil.clamp(
                stowMarginDeg.get(), ShooterHoodConstants.kMinStowMarginDeg, ShooterHoodConstants.kMaxStowMarginDeg);
    }

    /** Hood at or near the bottom stop: the shooter must not power the flywheel. */
    public boolean isDown() {
        return getPosition().lt(downThreshold());
    }

    private boolean inTrenchZone() {
        return FieldCalculations.inTrenchZone(RobotContainer.getDrivetrain().getState().Pose);
    }

    /** Fully down in auto (the 7457 paths pass the trench with it down) and near a trench in teleop; else stowed. */
    public Angle restAngle() {
        if (DriverStation.isAutonomous() || inTrenchZone()) {
            return ShooterHoodConstants.kReverseSoftLimit;
        }
        return stowAngle();
    }

    /** In teleop near a trench the hood stays fully down: aiming and shooting cannot raise it there. */
    private Angle trenchSafe(Angle angle) {
        return DriverStation.isTeleop() && inTrenchZone() ? ShooterHoodConstants.kReverseSoftLimit : angle;
    }

    public Angle getShotAngle() {
        return trenchSafe(Degrees.of(
                ShotCalculator.shotAngleDeg(RobotContainer.getDrivetrain().getState().Pose)));
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
        RobotStates.teleop.onTrue(goToPosition(this::restAngle));
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.auto_aim)
                .or(RobotStates.auto_shoot)
                .whileTrue(goToPosition(this::getShotAngle));
        RobotStates.actionSetShot.onTrue(goToPosition(() -> trenchSafe(Degrees.of(19))));
        RobotStates.auto_drop_hood.onTrue(goToPosition(() -> ShooterHoodConstants.kReverseSoftLimit));
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.auto_aim)
                .or(RobotStates.auto_shoot)
                .or(RobotStates.actionSetShot)
                .onFalse(goToPosition(this::restAngle));
        RobotStates.auto_drop_intake.onTrue(releaseIntake());
        RobotStates.actionZeroHood.onTrue(zero());
        RobotStates.testController.leftStickY.whileTrue(
                runEnd(() -> runVoltage(Volts.of(6 * RobotStates.testController.getDriveFwdPositive())), this::stop));
        // RobotStates.actionTestA.onTrue(run(() -> stop()).withName("stop"));
        // RobotStates.actionTestB.onTrue(goToPosition(() -> Degrees.of(10)).withName("10"));
        // RobotStates.actionTestX.onTrue(goToPosition(() -> Degrees.of(17)).withName("17"));
        // RobotStates.actionTestY.onTrue(goToPosition(() -> Degrees.of(25)).withName("25"));
    }
}
