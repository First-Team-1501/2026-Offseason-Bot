package org.supurdueper.robot2026.utils;

import static edu.wpi.first.units.Units.Meters;

import org.supurdueper.lib.utils.AllianceFlip;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldCalculations {

    public static Distance distanceToGoal(Pose2d robotPose) {
        Translation2d hubCenter = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return Meters.of(robotPose.getTranslation().getDistance(hubCenter));
    }

    


    
}
