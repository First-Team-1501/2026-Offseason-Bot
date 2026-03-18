package org.supurdueper.robot2026.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2026.utils.FieldCalculations;
import org.supurdueper.robot2026.utils.FieldConstants;

/**
 * The SwerveRequest::apply function runs in a fast (250hz on CAN FD) thread that is timed on the CTRE StatusSignal API.
 * This is the same thread that updates odometry. By extending this request to set the target angle to face calculated
 * against a specified point on the field instead of using SwerveRequest.FieldCentricFacingAngle, we can ensure that
 * we're updating the target rotation at 250hz and always utilizing the latest pose estimation.
 */
public class AutoAim extends FieldCentricFacingAngle {

    Rotation2d aimedTolerance = Rotation2d.fromDegrees(1.5);
    SwerveRequest xMode = new SwerveDriveBrake();
    Translation2d pointToFace;
    Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    double feedShotX = FieldConstants.LinesVertical.starting;
    Translation2d bottomHalfFieldFeed = new Translation2d(feedShotX, FieldConstants.LinesHorizontal.center / 2);
    Translation2d topHalfFieldFeed = new Translation2d(feedShotX, FieldConstants.LinesHorizontal.center * 1.5);

    public AutoAim() {}

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;

        if (FieldCalculations.ourZone(currentPose)) {
            this.pointToFace = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        } else if (FieldCalculations.bottomHalf(currentPose)) {
            this.pointToFace = AllianceFlip.apply(bottomHalfFieldFeed);
        } else {
            this.pointToFace = AllianceFlip.apply(topHalfFieldFeed);
        }

        DogLog.log("AutoAim/PointToFace", pointToFace);

        this.TargetDirection = pointToFace.minus(currentPose.getTranslation()).getAngle();
        if (!AllianceFlip.shouldFlip()) {
            this.TargetDirection = this.TargetDirection.rotateBy(Rotation2d.k180deg);
        }
        if (Math.abs(this.TargetDirection.minus(currentPose.getRotation()).getDegrees())
                < aimedTolerance.getDegrees()) {
            if (Math.abs(this.VelocityX) < 0.2 && Math.abs(this.VelocityY) < 0.2) {
                return xMode.apply(parameters, modulesToApply);
            }
        }
        return super.apply(parameters, modulesToApply);
    }
}
