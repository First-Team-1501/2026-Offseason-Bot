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

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {
    public static boolean tuningMode = false;
    public static boolean publishToNT = true;
    public static CANBus canivoreBus = new CANBus("canivore");
    public static CANBus rioBus = new CANBus("canivore");

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
        public static final Distance robotToBumperCenter = null;
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
        public static final CurrentLimitsConfigs kCurrentLimit = null;
    }


    public class HopperConstants {
        public static final CurrentLimitsConfigs kCurrentLimit = null;
        public static final Voltage kIntakeVoltage = null;
        public static final Voltage kPurgeVoltage = null;
        
    }

    public class IntakeConstants {

        public static final CurrentLimitsConfigs kCurrentLimit = null;
        public static final Voltage kIntakeVoltage = Volts.of(2);
        public static final Voltage kPurgeVoltage = Volts.of(-2);

    }
}
