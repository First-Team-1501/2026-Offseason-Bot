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
    public static final double loopPeriodSecs = 0.02;
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
        public static final double kSlowModeScalor = 0.85;
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
        public static final double kp = 800.0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        public static final Angle kForwardSoftLimit = Degrees.of(30);
        public static final Angle kReverseSoftLimit = Degrees.of(7);
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
        public static final double kP = 25.0;
        public static final double kS = 9.0;
        public static final double kV = 0.15;
        public static final double shooterGearRatio = 36.0 / 24.0;
        public static final AngularVelocity kVelocityTolerance = RPM.of(100);
        public static final AngularVelocity kShootRPM = RPM.of(1600);
        public static final AngularVelocity kIdleRPM = RPM.of(900);
        public static final AngularVelocity kRevRpm = RPM.of(1200);
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
        public static final AngularVelocity kIntakeSpeed = RPM.of(1800);
        public static final double kV = 0.0;
        public static final double kS = 12.0;
        public static final double kP = 8.0;
        public static final AngularVelocity velocityTolerance = RPM.of(50);
        public static final AngularVelocity kAgitateSpeed = RPM.of(-300);
        public static final Voltage kAgitateVoltage = Volts.of(0.5);
        public static final double kHasBallsCurrent = 0;
    }

    public class FeederConstants {
        public static final double kP = 6.5;
        public static final double kS = 3.7;
        public static final double kV = 0.07;
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
        public static final double kP = 12.0;
        public static final double kS = 3.0;
        public static final double kV = 0.1;
        public static final AngularVelocity velocityTolerance = RPM.of(60);
        public static final double kMaxAmps = 60.0;
        public static final double kGearRatio = 34.0 / 18.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(kMaxAmps).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity feedVelocity = RPM.of(1500);
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
        public static final AngularVelocity kShootVelocity = RPM.of(1700);
        public static final double kGearRatio = 24.0 / 12.0;
        public static final double kp = 10.0;
        public static final double ks = 7.5;
        public static final double kv = 0.1;
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

        private static void addPointToDistanceToShooterAngle(double distanceMeters, double angleDegrees) {
            distanceToShooterAngle.put(distanceMeters, angleDegrees);
        }

        private static void addPointToDistanceToShooterRPM(double distanceMeters, double velocityRPM) {
            distanceToShooterRPM.put(distanceMeters, velocityRPM);
        }

        static {
            addPointToDistanceToShooterAngle(1.75, 14.5);
            addPointToDistanceToShooterAngle(2.0, 15.5);
            addPointToDistanceToShooterAngle(2.25, 18.0);
            addPointToDistanceToShooterAngle(2.5, 18.5);
            addPointToDistanceToShooterAngle(2.75, 19.0);
            addPointToDistanceToShooterAngle(3.0, 21.0);
            addPointToDistanceToShooterAngle(3.25, 22.0);
            addPointToDistanceToShooterAngle(3.5, 22.5);
            addPointToDistanceToShooterAngle(3.75, 23.5);
            addPointToDistanceToShooterAngle(4.0, 24.0);
            addPointToDistanceToShooterAngle(4.25, 25.0);
            addPointToDistanceToShooterAngle(4.5, 25.5);
            addPointToDistanceToShooterAngle(4.75, 25.5);
            addPointToDistanceToShooterAngle(5, 25.5);
            addPointToDistanceToShooterAngle(5.25, 25.5);
            addPointToDistanceToShooterAngle(6, 30.0);

            addPointToDistanceToShooterRPM(1.75, 1350);
            addPointToDistanceToShooterRPM(2.0, 1375);
            addPointToDistanceToShooterRPM(2.25, 1395);
            addPointToDistanceToShooterRPM(2.5, 1450);
            addPointToDistanceToShooterRPM(2.75, 1490);
            addPointToDistanceToShooterRPM(3.0, 1525);
            addPointToDistanceToShooterRPM(3.25, 1575);
            addPointToDistanceToShooterRPM(3.5, 1675);
            addPointToDistanceToShooterRPM(3.75, 1700);
            addPointToDistanceToShooterRPM(4, 1750);
            addPointToDistanceToShooterRPM(4.25, 1775);
            addPointToDistanceToShooterRPM(4.5, 1850);
            addPointToDistanceToShooterRPM(4.75, 1900);
            addPointToDistanceToShooterRPM(5, 1975);
            addPointToDistanceToShooterRPM(5.25, 2025);
            addPointToDistanceToShooterRPM(6, 2125);
        }
    }
}
