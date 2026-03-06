package org.supurdueper.robot2026.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2026.utils.FieldConstants;

/**
 * The SwerveRequest::apply function runs in a fast (250hz on CAN FD) thread that is timed on the CTRE StatusSignal API.
 * This is the same thread that updates odometry. By extending this request to set the target angle to face calculated
 * against a specified point on the field instead of using SwerveRequest.FieldCentricFacingAngle, we can ensure that
 * we're updating the target rotation at 250hz and always utilizing the latest pose estimation.
 */
public class AimAtHub extends FieldCentricFacingAngle {

    Translation2d pointToFace;
    private final double shooterOffsetInches = 5;
    private Transform2d robotToShooterTransform =
            new Transform2d(new Translation2d(0, Units.inchesToMeters(shooterOffsetInches)), Rotation2d.kZero);

    public AimAtHub() {}

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        this.pointToFace = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d shooterPose = parameters.currentPose.transformBy(robotToShooterTransform);
        this.TargetDirection = pointToFace.minus(shooterPose.getTranslation()).getAngle();
        return super.apply(parameters, modulesToApply);
    }
}
