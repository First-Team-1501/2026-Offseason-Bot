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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {
    public static boolean tuningMode = true;
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

    public class ShooerHoodConstants {
        public static final Angle positionTolerance = Angle.ofBaseUnits(0, null);
        public static final Angle kAbsEncoderOffset = null;
        public static final double kAbsEncoderRatio = 0;
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        public static final Angle kForwardSoftLimit = null;
        public static final Angle kReverseSoftLimit = null;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity profileV = null;
        public static final AngularAcceleration profileA = null;
        public static final double kDegreesPerRotation = 0;
        ;
    }

    public class ShooterConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        public static final double kP = 5.0;
        public static final double kS = 6.0;
        public static final double kV = 0.08;
        public static final double shooterGearRatio = 30.0 / 24.0;
        public static final AngularVelocity kVelocityTolerance = RPM.of(50);
        public static final AngularVelocity kShootRPM = RPM.of(2200);
        public static final AngularVelocity kIdleRPM = RPM.of(1000);
    }

    public class HopperConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(0);
        public static final Voltage kPurgeVoltage = Volts.of(0);
        public static final double kV = 0;
        public static final double kS = 0;
        public static final double kP = 0;
        public static final AngularVelocity velocityTolerance = null;
    }

    public class FeederConstants {

        public static final double kP = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final AngularVelocity velocityTolerance = RPM.of(60);
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        public static final AngularVelocity feedVelocity = RPM.of(30);
        public static final AngularVelocity purgeVelocity = RotationsPerSecond.of(-20);
        public static final AngularVelocity idleVelocity = RotationsPerSecond.of(0);
    }

    public class IntakeConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(10);
        public static final Voltage kPurgeVoltage = Volts.of(-4);
        public static final Voltage kShootVoltage = Volts.of(10);
    }

    public class ClimberConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(120).withStatorCurrentLimitEnable(true);
        public static final GravityTypeValue GravityType = GravityTypeValue.Elevator_Static;
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKa = 0;
        public static final double profileKv = 0;
        public static final Distance kForwardSoftLimit = Inches.of(0);
        public static final Distance kReverseSoftLimit = Inches.of(0);
        public static final double kInchesPerRotation = 0;
        public static final AngularVelocity profileV = null;
        public static final AngularAcceleration profileA = null;
        public static final Distance kDropIntakePosition = null;
        public static final Distance kPrepClimbPosition = null;
        public static final Distance kHomePosition = null;
        public static Distance positionTolerance = Inches.of(0);
    }

    public class LightsConstants {
        public static double brightness = 0.5;
        public static int LEDCount = 7;
    }

    public class LookupTables {

        public static final InterpolatingDoubleTreeMap distanceToShooterAngle = new InterpolatingDoubleTreeMap();

        private static void addPointToDistanceToShooterAngle(double distanceInches, double angleDegrees) {
            distanceToShooterAngle.put(Units.inchesToMeters(distanceInches), Units.degreesToRotations(angleDegrees));
        }

        static {
            addPointToDistanceToShooterAngle(00, 51.0);
            addPointToDistanceToShooterAngle(12, 48.5);
            addPointToDistanceToShooterAngle(24, 42.5);
            addPointToDistanceToShooterAngle(36, 38.5);
            addPointToDistanceToShooterAngle(48, 35.5);
            addPointToDistanceToShooterAngle(60, 33.5);
            addPointToDistanceToShooterAngle(72, 31.5);
            addPointToDistanceToShooterAngle(84, 29.5);
            addPointToDistanceToShooterAngle(96, 27);
            addPointToDistanceToShooterAngle(108, 25.5);
            addPointToDistanceToShooterAngle(120, 24.5);
            addPointToDistanceToShooterAngle(132, 24.5);
            addPointToDistanceToShooterAngle(144, 23);
            addPointToDistanceToShooterAngle(156, 22.5);
            addPointToDistanceToShooterAngle(190, 22);
        }
    }
}
