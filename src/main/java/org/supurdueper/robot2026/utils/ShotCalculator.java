package org.supurdueper.robot2026.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.supurdueper.robot2026.Constants.LookupTables;
import org.supurdueper.robot2026.Constants.ShooterConstants;
import org.supurdueper.robot2026.Constants.ShooterHoodConstants;

/**
 * Flywheel speed and hood angle for the robot pose: hub shots from our zone, forwarding shots elsewhere.
 *
 * <p>Shot tuning from the dashboard, independent of {@code Constants.tuningMode}: with "Shot Tuning/Manual" on, the
 * shot uses "Shot Tuning/Speed (RPM)" and "Shot Tuning/Angle (Deg)", which start from the table values when the switch
 * is turned on. "Shot Tuning/Record" logs "Shot Tuning/Recorded Point" as [distance m, RPM, hood deg, forwarding 0/1].
 */
public final class ShotCalculator {

    private static final String kLutRpmScaleKey = "Shooter/LUT RPM Scale";
    private static final String kForwardRpmScaleKey = "Shooter/Forward RPM Scale";
    private static final String kManualKey = "Shot Tuning/Manual";
    private static final String kManualRpmKey = "Shot Tuning/Speed (RPM)";
    private static final String kManualAngleKey = "Shot Tuning/Angle (Deg)";
    private static final String kRecordKey = "Shot Tuning/Record";
    private static final String kDistanceKey = "Shot Tuning/Distance (m)";
    private static final double kMaxManualRpm = 3500;

    private static boolean wasManual = false;

    static {
        SmartDashboard.putNumber(kLutRpmScaleKey, ShooterConstants.kLutRpmScale);
        SmartDashboard.putNumber(kForwardRpmScaleKey, ShooterConstants.kForwardRpmScale);
        SmartDashboard.putBoolean(kManualKey, false);
        SmartDashboard.putNumber(kManualRpmKey, 0);
        SmartDashboard.putNumber(kManualAngleKey, ShooterHoodConstants.kReverseSoftLimit.in(Degrees));
        SmartDashboard.putBoolean(kRecordKey, false);
    }

    private ShotCalculator() {}

    public static double shotRpm(Pose2d robotPose) {
        if (isManual()) {
            return MathUtil.clamp(SmartDashboard.getNumber(kManualRpmKey, 0), 0, kMaxManualRpm);
        }
        return tableRpm(robotPose);
    }

    public static double shotAngleDeg(Pose2d robotPose) {
        if (isManual()) {
            return MathUtil.clamp(
                    SmartDashboard.getNumber(kManualAngleKey, ShooterHoodConstants.kReverseSoftLimit.in(Degrees)),
                    ShooterHoodConstants.kReverseSoftLimit.in(Degrees),
                    ShooterHoodConstants.kForwardSoftLimit.in(Degrees));
        }
        return tableAngleDeg(robotPose);
    }

    public static double tableRpm(Pose2d robotPose) {
        if (FieldCalculations.isForwarding(robotPose)) {
            return LookupTables.distanceToForwardRPM.get(distanceMeters(robotPose))
                    * SmartDashboard.getNumber(kForwardRpmScaleKey, ShooterConstants.kForwardRpmScale);
        }
        return LookupTables.distanceToShooterRPM.get(distanceMeters(robotPose))
                * SmartDashboard.getNumber(kLutRpmScaleKey, ShooterConstants.kLutRpmScale);
    }

    public static double tableAngleDeg(Pose2d robotPose) {
        if (FieldCalculations.isForwarding(robotPose)) {
            return LookupTables.distanceToForwardAngle.get(distanceMeters(robotPose));
        }
        return LookupTables.distanceToShooterAngle.get(distanceMeters(robotPose));
    }

    /** Distance the tables use: robot center to the hub from our zone, to the forwarding point elsewhere. */
    public static double distanceMeters(Pose2d robotPose) {
        return FieldCalculations.distanceToAimPoint(robotPose).in(Meters);
    }

    /** Call once per loop. */
    public static void periodic(Pose2d robotPose) {
        double distance = distanceMeters(robotPose);
        boolean forwarding = FieldCalculations.isForwarding(robotPose);
        boolean manual = isManual();
        if (manual && !wasManual) {
            SmartDashboard.putNumber(kManualRpmKey, tableRpm(robotPose));
            SmartDashboard.putNumber(kManualAngleKey, tableAngleDeg(robotPose));
        }
        wasManual = manual;
        SmartDashboard.putNumber(kDistanceKey, distance);
        DogLog.log("Shooter/Forwarding", forwarding);
        DogLog.log("Shooter/Distance To Aim Point", distance);
        DogLog.log("Shot Tuning/Manual", manual);
        if (SmartDashboard.getBoolean(kRecordKey, false)) {
            SmartDashboard.putBoolean(kRecordKey, false);
            double rpm = shotRpm(robotPose);
            double angle = shotAngleDeg(robotPose);
            DogLog.log("Shot Tuning/Recorded Point", new double[] {distance, rpm, angle, forwarding ? 1 : 0});
            System.out.printf(
                    "Shot Tuning point: %.2f m, %.0f RPM, %.1f deg%s%n",
                    distance, rpm, angle, forwarding ? " (forwarding)" : "");
        }
    }

    private static boolean isManual() {
        return SmartDashboard.getBoolean(kManualKey, false);
    }
}
