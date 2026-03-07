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
import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {
    public static boolean tuningMode = false;
    public static boolean publishToNT = true;
    public static CANBus canivoreBus = new CANBus("canivore");
    public static CANBus rioBus = new CANBus("rio");

    public static final class DriverConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadzone = 0.1;
        public static final ExpCurve kLeftStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kRightStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kTriggerCurve = new ExpCurve(1, 0, 1, kDeadzone);
        public static final double kSlowModeScalor = 0.45;
        public static final double kDefaultTurnScalor = 0.75;
        public static final double kTurboModeScalor = 1;
    }

    public static final class DriveConstants {
        public static final double headingKp = 5.0;
        public static final double headingKi = 0;
        public static final double headingKd = 0.0;
        public static final double translationKp = 7.0;
        public static final double translationKi = 0;
        public static final double translationKd = 0.1;
        public static final AngularVelocity rotationClosedLoopDeadband = RadiansPerSecond.of(0.05);
        public static final LinearVelocity translationClosedLoopDeadband = MetersPerSecond.of(0.01);
        public static final Translation2d robotToBumperCenter = null;
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    public class ShooterHoodConstants {
        public static final double kp = 1000.0;
        public static final double ki = 2000.0;
        public static final double kd = 8.0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        public static final Angle kForwardSoftLimit = Degrees.of(37);
        public static final Angle kReverseSoftLimit = Degrees.of(19);
        public static final Voltage kPeakForwardVoltage = Volts.of(3);
        public static final Voltage kPeakReverseVoltage = Volts.of(-3);
        public static final Angle kZeroPosition = Degrees.of(19);
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity profileV = RotationsPerSecond.of(0);
        public static final AngularAcceleration profileA = RotationsPerSecondPerSecond.of(0);
        public static final double gearRatio = 300.0 / 20.0 * 53.0 / 10.0;
        public static Angle positionTolerance = Degrees.of(0.3);
    }

    public class ShooterConstants {
        public static final double kMaxAmps = 60.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final double kP = 8.0;
        public static final double kS = 6.0;
        public static final double kV = 0.08;
        public static final double shooterGearRatio = 30.0 / 24.0;
        public static final AngularVelocity kVelocityTolerance = RPM.of(50);
        public static final AngularVelocity kShootRPM = RPM.of(1600);
        public static final AngularVelocity kIdleRPM = RPM.of(1000);
    }

    public class HopperConstants {
        public static final double kMaxAmps = 80;
        public static final double gearRatio = 30.0 / 14.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(10);
        public static final Voltage kPurgeVoltage = Volts.of(0);
        public static final AngularVelocity kIntakeSpeed = RPM.of(1800);
        public static final double kV = 0.0;
        public static final double kS = 12.0;
        public static final double kP = 8.0;
        public static final AngularVelocity velocityTolerance = RPM.of(50);
        public static final AngularVelocity kAgitateSpeed = RPM.of(-300);
    }

    public class FeederConstants {
        public static final double kP = 5.0;
        public static final double kS = 3.5;
        public static final double kV = 0;
        public static final AngularVelocity velocityTolerance = RPM.of(60);
        public static final double kMaxAmps = 60.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity feedVelocity = RPM.of(2000);
        public static final AngularVelocity purgeVelocity = RPM.of(-500);
        public static final AngularVelocity idleVelocity = RPM.of(1000);
    }

    public class IntakeConstants {
        public static final double kMaxAmps = 60;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(6);
        public static final Voltage kPurgeVoltage = Volts.of(-8);
        public static final Voltage kShootVoltage = Volts.of(6);
        public static final AngularVelocity kVelocityTolerance = RPM.of(50);
        public static final AngularVelocity kIntakeVelocity = RPM.of(1700);
        public static final AngularVelocity kPurgeVelocity = RPM.of(-4);
        public static final AngularVelocity kShootVelocity = RPM.of(1700);
        public static final double kGearRatio = 18.0 / 12.0;
        public static final double kp = 10.0;
        public static final double ks = 8.0;
        public static final double kv = 0.12;
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
        public static int LEDCount = 7;
    }

    public class LookupTables {

        public static final InterpolatingDoubleTreeMap distanceToShooterAngle = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToShooterRPM = new InterpolatingDoubleTreeMap();

        private static void addPointToDistanceToShooterAngle(double distanceMeters, double angleDegrees) {
            distanceToShooterAngle.put(distanceMeters, angleDegrees);
        }

        private static void addPointToDistanceToShooterRPM(double distanceMeters, double velocityRPM) {
            distanceToShooterRPM.put(distanceMeters, velocityRPM);
        }

        static {
            addPointToDistanceToShooterAngle(1.6, 19.0);
            addPointToDistanceToShooterAngle(2.0, 19.0);
            addPointToDistanceToShooterAngle(2.25, 20.0);
            addPointToDistanceToShooterAngle(2.5, 22.0);
            addPointToDistanceToShooterAngle(2.75, 24.0);
            addPointToDistanceToShooterAngle(3.0, 26.5);
            addPointToDistanceToShooterAngle(3.25, 28.0);
            addPointToDistanceToShooterAngle(3.5, 29.0);
            addPointToDistanceToShooterAngle(5.3, 32.0);

            addPointToDistanceToShooterRPM(1.6, 1550);
            addPointToDistanceToShooterRPM(2.0, 1625);
            addPointToDistanceToShooterRPM(2.25, 1650);
            addPointToDistanceToShooterRPM(2.5, 1700);
            addPointToDistanceToShooterRPM(2.75, 1700);
            addPointToDistanceToShooterRPM(3.0, 1700);
            addPointToDistanceToShooterRPM(3.25, 1750);
            addPointToDistanceToShooterRPM(3.5, 1800);
            addPointToDistanceToShooterRPM(5.3, 2000);
        }
    }
}
