// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Constants.LookupTables;
import org.supurdueper.robot2026.Constants.ShooterConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.utils.FieldCalculations;

public class Shooter extends VelocitySubsystem implements SupurdueperSubsystem {

    private final LoggedTunableNumber shooterVelocity;
    /** Creates a new VelocityTest. */
    public Shooter() {
        config.Feedback.SensorToMechanismRatio = ShooterConstants.shooterGearRatio;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakForwardDutyCycle = 0.0;
        configureMotors();
        // Manually create followers since we have more than two
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_TWO, motor, false);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_THREE, motor, false);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FOUR, motor, true);
        TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FIVE, motor, true);
        Robot.add(this);
        shooterVelocity = new LoggedTunableNumber("Shot Tuning/Speed (RPM)");
        shooterVelocity.initDefault(1500);
    }

    @Override
    protected boolean isAtVelocity() {
        if (getVelocity().lt(RPM.of(1000))) return false;
        return super.isAtVelocity();
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Shooter/Current RPM", getVelocity().in(RPM));
        DogLog.log("Shooter/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Shooter/At Velocity", isAtVelocity());
        DogLog.log("Shooter/Current", motor.getTorqueCurrent().getValue().in(Amps));
        DogLog.log("Battery Voltage", RobotController.getBatteryVoltage());
    }

    @Override
    public CanId canIdLeader() {
        return CanId.SHOOTER_ONE;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return ShooterConstants.kCurrentLimit;
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
                .withKP(ShooterConstants.kP)
                .withKI(0)
                .withKD(0)
                .withKS(ShooterConstants.kS)
                .withKV(ShooterConstants.kV)
                .withKA(0);
    }

    @Override
    public AngularVelocity velocityTolerance() {
        return ShooterConstants.kVelocityTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return null;
    }

    public void test() {
        runVoltage(Volts.of(2));
    }

    private AngularVelocity getShotVelocity() {
        if (Constants.tuningMode) {
            return RPM.of(shooterVelocity.get());
        } else {
            double distanceToGoalMeters = FieldCalculations.distanceToGoal(
                            RobotContainer.getDrivetrain().getState().Pose)
                    .in(Meters);
            return RPM.of(LookupTables.distanceToShooterRPM.get(distanceToGoalMeters));
        }
    }

    @Override
    public void bindCommands() {
        RobotStates.auto_rev.onTrue(goToVelocity(() -> ShooterConstants.kRevRpm));
        RobotStates.actionAim.or(RobotStates.actionShoot).onTrue(goToVelocity(this::getShotVelocity));
        RobotStates.auto_aim
                .or(RobotStates.auto_shoot)
                .or(RobotStates.actionSetShot)
                .onTrue(goToVelocity(this::getShotVelocity));
        RobotStates.actionAim
                .or(RobotStates.actionShoot)
                .or(RobotStates.actionSetShot)
                .onFalse(goToVelocity(() -> ShooterConstants.kIdleRPM));
        RobotStates.actionSetShot.whileTrue(goToVelocity(() -> RPM.of(1490)));
        // RobotStates.testController.B.onTrue(goToVelocity(() -> RPM.of(500)));
        // RobotStates.testController.X.onTrue(goToVelocity(() -> RPM.of(1000)));
        // RobotStates.testController.Y.onTrue(goToVelocity(() -> RPM.of(2000)));
        // RobotStates.testController.A.onTrue(run(this::stop));
    }

    // Manually creating followers in constructor
    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
