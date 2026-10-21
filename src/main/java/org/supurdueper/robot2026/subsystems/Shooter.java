// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.VelocitySubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants.ShooterConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.utils.ShotCalculator;

public class Shooter extends VelocitySubsystem implements SupurdueperSubsystem {

    private final List<TalonFX> allMotors = new ArrayList<>();

    /** Creates a new VelocityTest. */
    public Shooter() {
        config.Feedback.SensorToMechanismRatio = ShooterConstants.shooterGearRatio;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.kMaxAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        configureMotors();
        allMotors.add(motor);
        // Manually create followers since we have more than two
        allMotors.add(TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_TWO, motor, false));
        // Motor 1 sits on the opposite side from the leader (12): with Aligned it braked the flywheel at the current
        // limit (2026-10-17 per-motor log)
        allMotors.add(TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_THREE, motor, true));
        allMotors.add(TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FOUR, motor, true));
        allMotors.add(TalonFXFactory.createPermanentFollowerTalon(CanId.SHOOTER_FIVE, motor, true));
        Robot.add(this);
    }

    /** The flywheel is never powered while the hood is down; commands keep running and resume when it rises. */
    private boolean hoodInterlock() {
        return RobotContainer.getShooterHood().isDown();
    }

    @Override
    protected void setVelocity(AngularVelocity angularvelocity) {
        if (hoodInterlock()) {
            stop();
            return;
        }
        super.setVelocity(angularvelocity);
    }

    @Override
    protected void setVelocity(double angularvelocity) {
        if (hoodInterlock()) {
            stop();
            return;
        }
        super.setVelocity(angularvelocity);
    }

    @Override
    protected void runVoltage(Voltage volts) {
        if (hoodInterlock()) {
            stop();
            return;
        }
        super.runVoltage(volts);
    }

    @Override
    protected boolean isAtVelocity() {
        if (hoodInterlock()) return false;
        if (getVelocity().lt(RPM.of(1000))) return false;
        return super.isAtVelocity();
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Shooter/Current RPM", getVelocity().in(RPM));
        DogLog.log("Shooter/Target RPM", getSetpoint().in(RPM));
        DogLog.log("Shooter/At Velocity", isAtVelocity());
        DogLog.log("Shooter/Hood Interlock", hoodInterlock());
        DogLog.log("Shooter/Current", motor.getTorqueCurrent().getValue().in(Amps));
        DogLog.log("Battery Voltage", RobotController.getBatteryVoltage());
        ShotCalculator.periodic(RobotContainer.getDrivetrain().getState().Pose);
        for (TalonFX shooterMotor : allMotors) {
            String key = "Shooter/Motor " + shooterMotor.getDeviceID();
            double rpm = shooterMotor.getVelocity().getValue().in(RPM);
            double torqueCurrentAmps =
                    shooterMotor.getTorqueCurrent().getValue().in(Amps);
            DogLog.log(key + "/RPM", rpm);
            DogLog.log(key + "/Torque Current", torqueCurrentAmps);
            DogLog.log(
                    key + "/Supply Current",
                    shooterMotor.getSupplyCurrent().getValue().in(Amps));
            DogLog.log(
                    key + "/Voltage", shooterMotor.getMotorVoltage().getValue().in(Volts));
            // Torque against its own rotation: braking. A follower doing this while the others drive the flywheel has
            // the wrong Aligned/Opposed flag.
            DogLog.log(key + "/Braking", rpm * torqueCurrentAmps < 0 && Math.abs(torqueCurrentAmps) > 10);
        }
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
        return false;
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
        return RPM.of(ShotCalculator.shotRpm(RobotContainer.getDrivetrain().getState().Pose));
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
        RobotStates.actionSetShot.whileTrue(goToVelocity(() -> RPM.of(1425)));
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
