// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.supurdueper.robot2026;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {
    public static final double loopPeriodSecs = 0.02;
    // true for tuning sessions (dashboard gains under /Tuning); false for events
    public static boolean tuningMode = true;
    public static boolean publishToNT = true;
    public static CANBus upperBus = new CANBus("Upper Bot CANivore");
    public static CANBus lowerBus = new CANBus("Lower Bot Canivore");
    public static CANBus rioBus = new CANBus("rio");

    public static final class DriverConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadzone = 0.1;
        public static final ExpCurve kLeftStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kRightStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kTriggerCurve = new ExpCurve(1, 0, 1, kDeadzone);
        public static final double kSlowModeScalor = 0.85;
        public static final double kDefaultTurnScalor = 0.75;
        public static final double kTurboModeScalor = 1;
        public static final int kDriveStickPort = 0;
        public static final int kRotationStickPort = 1;
        public static final int kTestControllerPort = 3;
        public static final double kStickDeadband = 0.05;
        public static final double kStickExponent = 3.0;
    }

    public static final class DriveConstants {
        public static final double headingKp = 15.0;
        public static final double headingKi = 0.0;
        // kD 0.2 -> 0.6 from aim step responses in the 2026-10-17 logs (less overshoot, same settle time)
        public static final double headingKd = 0.6;
        public static final double translationKp = 7.0;
        public static final double translationKi = 0;
        public static final double translationKd = 0.1;
        public static final AngularVelocity rotationClosedLoopDeadband = RadiansPerSecond.of(0.00);
        public static final LinearVelocity translationClosedLoopDeadband = MetersPerSecond.of(0.01);
        public static final Translation2d robotToBumperCenter = null;
    }

    public static final class VisionConstants {
        public static final List<String> kLimelightNames = List.of("limelight");
        // Robot-space camera pose sent to the Limelight from code (overrides the web UI); same order and signs as the
        // web UI fields LL Forward/Right/Up (m) and LL Roll/Pitch/Yaw (deg). Camera behind center, looking backward.
        public static final double kCameraForwardMeters = -0.28;
        public static final double kCameraRightMeters = 0.13;
        public static final double kCameraUpMeters = 0.199;
        public static final double kCameraRollDegrees = 0.0;
        public static final double kCameraPitchDegrees = 25.0;
        public static final double kCameraYawDegrees = 180.0;
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    public class ShooterHoodConstants {
        public static final double kp = 800.0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        // Measured top of travel on the 1501 hood (Tuner X position, current conversion): 0.115234 rot = 41.5 deg
        public static final Angle kForwardSoftLimit = Rotations.of(0.115234);
        public static final Angle kReverseSoftLimit = Degrees.of(7);
        // The flywheel must not be powered with the hood fully down. Rest (stow) angle is kReverseSoftLimit plus this
        // margin; below kReverseSoftLimit plus half the margin the hood counts as down and the flywheel is unpowered
        // (ShooterHood.isDown). Adjustable live in tuning mode as "ShooterHood/Stow Margin (Deg)", clamped below.
        public static final double kStowMarginDeg = 1.5;
        public static final double kMinStowMarginDeg = 0.5;
        public static final double kMaxStowMarginDeg = 5.0;
        public static final Voltage kPeakForwardVoltage = Volts.of(12);
        public static final Voltage kPeakReverseVoltage = Volts.of(-12);
        public static final Angle kZeroPosition = Degrees.of(6.837);
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(50).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity profileV = RotationsPerSecond.of(0);
        public static final AngularAcceleration profileA = RotationsPerSecondPerSecond.of(0);
        public static final double gearRatio = 15.0 / 1.0 * 34.0 / 18.0 * 140.0 / 12.0;
        public static Angle positionTolerance = Degrees.of(0.3);
    }

    public class ShooterConstants {
        public static final double kMaxAmps = 50.0;
        public static final CurrentLimitsConfigs kCurrentLimit = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kMaxAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);
        // VelocityVoltage gains, volts per mechanism rotation/sec. kS 0.55 was fitted while motor 1 braked the
        // flywheel;
        // with that fixed most of the drag is gone, so kS is back to 0.2 until it is refit from new logs.
        public static final double kP = 0.25;
        public static final double kS = 0.2;
        public static final double kV = 0.18;
        public static final double shooterGearRatio = 36.0 / 24.0;
        public static final AngularVelocity kVelocityTolerance = RPM.of(100);
        public static final AngularVelocity kShootRPM = RPM.of(1600);
        public static final AngularVelocity kIdleRPM = RPM.of(900);
        public static final AngularVelocity kRevRpm = RPM.of(1200);
        // Multiplier on LookupTables.distanceToShooterRPM; adjustable live as SmartDashboard "Shooter/LUT RPM Scale".
        public static final double kLutRpmScale = 1.10;
        // Multiplier on LookupTables.distanceToForwardRPM; adjustable live as SmartDashboard "Shooter/Forward RPM
        // Scale".
        public static final double kForwardRpmScale = 1.0;
        // Hub shots (drive stick 18) do not feed from farther than this; set from shot testing
        public static final double kMaxHubShotDistanceMeters = 4.0;
    }

    public class HopperConstants {
        public static final double kMaxAmps = 70;
        public static final double gearRatio = 40.0 / 18.0;
        public static final CurrentLimitsConfigs kCurrentLimit = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kMaxAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(10);
        public static final Voltage kPurgeVoltage = Volts.of(0);
        // 1900 keeps the feed rate the 2026-10-17 shots ran at (1800 with the old gains actually ran about 1900)
        public static final AngularVelocity kIntakeSpeed = RPM.of(1900);
        // VelocityVoltage gains, volts per mechanism rotation/sec. kS and kV fitted from the 2026-10-17 logs.
        public static final double kV = 0.22;
        public static final double kS = 0.25;
        public static final double kP = 0.2;
        public static final AngularVelocity velocityTolerance = RPM.of(50);
        public static final AngularVelocity kAgitateSpeed = RPM.of(-300);
        public static final Voltage kAgitateVoltage = Volts.of(0.5);
        public static final double kHasBallsCurrent = 0;
    }

    public class FeederConstants {
        // VelocityVoltage gains, volts per mechanism rotation/sec. kS and kV fitted from the 2026-10-17 logs.
        public static final double kP = 0.2;
        public static final double kS = 0.085;
        public static final double kV = 0.154;
        public static final AngularVelocity velocityTolerance = RPM.of(60);
        public static final double kMaxAmps = 60.0;
        public static final double kGearRatio = 24.0 / 18.0;
        public static final CurrentLimitsConfigs kCurrentLimit = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kMaxAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimitEnable(true);
        public static final AngularVelocity feedVelocity = RPM.of(2000);
        public static final AngularVelocity purgeVelocity = RPM.of(-500);
        public static final AngularVelocity idleVelocity = RPM.of(500);
    }

    public class UptakeConstants {
        // VelocityVoltage gains, volts per mechanism rotation/sec. Fitted from the 2026-10-17 logs (one operating
        // point, 1500 RPM, plus coast-down friction).
        public static final double kP = 0.2;
        public static final double kS = 0.25;
        public static final double kV = 0.218;
        public static final AngularVelocity velocityTolerance = RPM.of(60);
        public static final double kMaxAmps = 60.0;
        public static final double kGearRatio = 34.0 / 18.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity setShotVelocity = RPM.of(1000);
        public static final AngularVelocity forwardVelocity = RPM.of(2000);
        public static final AngularVelocity purgeVelocity = RPM.of(-500);
        public static final AngularVelocity idleVelocity = RPM.of(500);
    }

    public class IntakeConstants {
        public static final double kMaxAmps = 70;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(6);
        public static final Voltage kPurgeVoltage = Volts.of(-8);
        public static final Voltage kShootVoltage = Volts.of(6);
        public static final AngularVelocity kVelocityTolerance = RPM.of(50);
        public static final AngularVelocity kIntakeVelocity = RPM.of(4000);
        public static final AngularVelocity kPurgeVelocity = RPM.of(-4);
        // While shooting; negative = opposite of the intake direction
        public static final AngularVelocity kShootVelocity = RPM.of(-1700);
        public static final double kGearRatio = 24.0 / 12.0;
        // VelocityVoltage gains, volts per mechanism rotation/sec. kv from the fitted back-EMF, ks for the ball load
        // (2026-10-17 logs). 4000 RPM intake is beyond what the motors reach, so it runs at full output either way.
        public static final double kp = 0.2;
        public static final double ks = 1.0;
        public static final double kv = 0.185;
    }

    public class ClimberConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(120).withStatorCurrentLimitEnable(true);
        public static final GravityTypeValue GravityType = GravityTypeValue.Elevator_Static;
        public static final double kp = 10.0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        public static final double kForwardSoftLimit = 80;
        public static final double kReverseSoftLimit = 0;
        public static final double kInchesPerRotation = 0;
        public static final AngularVelocity profileV = RotationsPerSecond.of(0);
        public static final AngularAcceleration profileA = RotationsPerSecondPerSecond.of(0);
        public static final double kDropIntakePosition = 80;
        public static final double kClimbPosition = 14;
        public static final double kHomePosition = 0;
        public static double positionTolerance = 0;
    }

    public class LightsConstants {
        public static double brightness = 0.5;
        public static int LEDCount = 21;
    }

    public class LookupTables {

        public static final InterpolatingDoubleTreeMap distanceToShooterAngle = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToShooterRPM = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToUptakeRPM = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToTime = new InterpolatingDoubleTreeMap();
        // Forwarding: distance from robot to FieldCalculations.aimPoint (forwarding point in our zone)
        public static final InterpolatingDoubleTreeMap distanceToForwardAngle = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToForwardRPM = new InterpolatingDoubleTreeMap();

        private static void addPointToDistanceToForwardAngle(double distanceMeters, double angleDegrees) {
            distanceToForwardAngle.put(distanceMeters, angleDegrees);
        }

        private static void addPointToDistanceToForwardRPM(double distanceMeters, double velocityRPM) {
            distanceToForwardRPM.put(distanceMeters, velocityRPM);
        }

        private static void addPointToDistanceToShooterAngle(double distanceMeters, double angleDegrees) {
            distanceToShooterAngle.put(distanceMeters, angleDegrees);
        }

        private static void addPointToDistanceToUptakeRPM(double distanceMeters, double velocityRPM) {
            distanceToUptakeRPM.put(distanceMeters, velocityRPM);
        }

        private static void addPointToDistanceToShooterRPM(double distanceMeters, double velocityRPM) {
            distanceToShooterRPM.put(distanceMeters, velocityRPM);
        }

        private static void addPointToDistanceToTime(double distanceMeters, double timeSeconds) {
            distanceToTime.put(distanceMeters, timeSeconds);
        }

        static {
            addPointToDistanceToShooterAngle(1.75, 13.5);
            addPointToDistanceToShooterAngle(2.0, 15.0);
            addPointToDistanceToShooterAngle(2.25, 16.5);
            addPointToDistanceToShooterAngle(2.5, 17.5);
            addPointToDistanceToShooterAngle(2.75, 19.0);
            addPointToDistanceToShooterAngle(3.0, 21.0);
            addPointToDistanceToShooterAngle(3.25, 21.0);
            addPointToDistanceToShooterAngle(3.5, 21.0);
            addPointToDistanceToShooterAngle(3.75, 21.0);
            addPointToDistanceToShooterAngle(4.0, 21.0);
            addPointToDistanceToShooterAngle(4.25, 21.0);
            addPointToDistanceToShooterAngle(4.5, 21.0);
            addPointToDistanceToShooterAngle(4.75, 21.0);
            addPointToDistanceToShooterAngle(5, 22.0);
            addPointToDistanceToShooterAngle(5.25, 22.0);
            addPointToDistanceToShooterAngle(6, 30.0);

            addPointToDistanceToShooterRPM(1.75, 1340);
            addPointToDistanceToShooterRPM(2.0, 1365);
            addPointToDistanceToShooterRPM(2.25, 1390);
            addPointToDistanceToShooterRPM(2.5, 1415);
            addPointToDistanceToShooterRPM(2.75, 1440);
            addPointToDistanceToShooterRPM(3.0, 1475);
            addPointToDistanceToShooterRPM(3.25, 1550);
            addPointToDistanceToShooterRPM(3.5, 1675);
            addPointToDistanceToShooterRPM(3.75, 1775);
            addPointToDistanceToShooterRPM(4, 1800);
            addPointToDistanceToShooterRPM(4.25, 1875);
            addPointToDistanceToShooterRPM(4.5, 1900);
            addPointToDistanceToShooterRPM(4.75, 1925);
            addPointToDistanceToShooterRPM(5, 1975);
            addPointToDistanceToShooterRPM(5.25, 1975);
            addPointToDistanceToShooterRPM(6, 2075);

            addPointToDistanceToUptakeRPM(0, 1500);
            addPointToDistanceToUptakeRPM(2.75, 1500);
            addPointToDistanceToUptakeRPM(3.25, 1000);
            addPointToDistanceToUptakeRPM(4, 1000);
            addPointToDistanceToUptakeRPM(4.25, 800);
            addPointToDistanceToUptakeRPM(5.25, 800);
            addPointToDistanceToUptakeRPM(5.5, 2000);

            // Untuned starting points: flattest hood, RPM sized to land past the forwarding point
            addPointToDistanceToForwardAngle(2.0, 30.0);
            addPointToDistanceToForwardAngle(8.0, 30.0);

            addPointToDistanceToForwardRPM(2.0, 1750);
            addPointToDistanceToForwardRPM(3.0, 2000);
            addPointToDistanceToForwardRPM(4.0, 2200);
            addPointToDistanceToForwardRPM(5.0, 2400);
            addPointToDistanceToForwardRPM(6.0, 2575);
            addPointToDistanceToForwardRPM(7.0, 2750);
            addPointToDistanceToForwardRPM(8.0, 2900);
        }
    }
}
