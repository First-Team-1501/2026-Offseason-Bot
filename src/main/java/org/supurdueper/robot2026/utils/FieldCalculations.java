package org.supurdueper.robot2026.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import org.supurdueper.lib.utils.AllianceFlip;

public class FieldCalculations {

    // Forwarding targets in our alliance zone (blue frame), one per field half
    private static final Translation2d bottomHalfForwardPoint =
            new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center / 2);
    private static final Translation2d topHalfForwardPoint =
            new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center * 1.5);

    public static Distance distanceToGoal(Pose2d robotPose) {
        Translation2d hubCenter = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return Meters.of(robotPose.getTranslation().getDistance(hubCenter));
    }

    public static Boolean ourZone(Pose2d robotPose) {
        return AllianceFlip.applyX(robotPose.getX()) < FieldConstants.LinesVertical.neutralZoneNear;
    }

    public static Boolean bottomHalf(Pose2d robotPose) {
        return (AllianceFlip.applyY(robotPose.getY()) < FieldConstants.LinesHorizontal.center);
    }

    /** Outside our zone, shots forward fuel back into our zone instead of scoring. */
    public static Boolean isForwarding(Pose2d robotPose) {
        return !ourZone(robotPose);
    }

    /** Where to aim: the hub from our zone, otherwise the forwarding point on the robot's half of the field. */
    public static Translation2d aimPoint(Pose2d robotPose) {
        if (ourZone(robotPose)) {
            return AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        }
        return AllianceFlip.apply(bottomHalf(robotPose) ? bottomHalfForwardPoint : topHalfForwardPoint);
    }

    public static Distance distanceToAimPoint(Pose2d robotPose) {
        return Meters.of(robotPose.getTranslation().getDistance(aimPoint(robotPose)));
    }

    // Trench zone: the trench depth plus half the robot length and hood travel time at full speed (x), and a small
    // margin beside the opening (y)
    private static final double kTrenchZoneMarginXMeters = 1.0;
    private static final double kTrenchZoneMarginYMeters = 0.2;

    /** Under or approaching any of the four trenches (both alliance sides), where the hood must be fully down. */
    public static boolean inTrenchZone(Pose2d robotPose) {
        double halfZoneDepth = FieldConstants.LeftTrench.depth / 2.0 + kTrenchZoneMarginXMeters;
        boolean trenchX = Math.abs(robotPose.getX() - FieldConstants.LinesVertical.hubCenter) < halfZoneDepth
                || Math.abs(robotPose.getX() - FieldConstants.LinesVertical.oppHubCenter) < halfZoneDepth;
        boolean leftTrenchY = robotPose.getY()
                > FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth - kTrenchZoneMarginYMeters;
        boolean rightTrenchY = robotPose.getY() < FieldConstants.RightTrench.openingWidth + kTrenchZoneMarginYMeters;
        return trenchX && (leftTrenchY || rightTrenchY);
    }
}
