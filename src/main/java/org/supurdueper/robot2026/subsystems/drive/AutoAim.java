package org.supurdueper.robot2026.subsystems.drive;

import static org.supurdueper.robot2026.Constants.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import org.supurdueper.robot2026.utils.FieldCalculations;

/**
 * The SwerveRequest::apply function runs in a fast (250hz on CAN FD) thread that is timed on the CTRE StatusSignal API.
 * This is the same thread that updates odometry. By extending this request to set the target angle to face calculated
 * against a specified point on the field instead of using SwerveRequest.FieldCentricFacingAngle, we can ensure that
 * we're updating the target rotation at 250hz and always utilizing the latest pose estimation.
 *
 * <p>While shooting ({@link #withLockWhenAimed}) the wheels X-lock once aimed, so other robots cannot push the robot
 * off target: locked within {@code kAimLockEnterDeg} while neither translating nor turning; released above
 * {@code kAimLockExitDeg} (the heading controller re-aims, then locks again) or when the driver translates.
 */
public class AutoAim extends FieldCentricFacingAngle {

    SwerveRequest xMode = new SwerveDriveBrake();
    Translation2d pointToFace;
    // apply() runs at 250 Hz; DogLog gets every 5th call (50 Hz) to keep its queue from overflowing
    private static final int kDogLogDecimation = 5;
    private int applyCount = 0;
    private volatile boolean lockWhenAimed = false;
    private volatile boolean locked = false;

    // Heading tuning telemetry at the request rate (DataLog, not DogLog)
    private final DoubleLogEntry headingTarget;
    private final DoubleLogEntry headingMeasured;
    private final DoubleLogEntry headingError;
    private final DoubleLogEntry headingOutput;
    private final BooleanLogEntry headingLocked;

    public AutoAim() {
        // TargetDirection is field-relative (blue origin) on both alliances; translation still follows
        // the ForwardPerspective set by the caller.
        this.TargetDirectionPerspective = TargetDirectionPerspectiveValue.BlueAlliance;
        DataLog log = DataLogManager.getLog();
        headingTarget = new DoubleLogEntry(log, "/Tuning/Heading/Target");
        headingMeasured = new DoubleLogEntry(log, "/Tuning/Heading/Measured");
        headingError = new DoubleLogEntry(log, "/Tuning/Heading/Error");
        headingOutput = new DoubleLogEntry(log, "/Tuning/Heading/Output");
        headingLocked = new BooleanLogEntry(log, "/Tuning/Heading/Locked");
    }

    /** X-lock the wheels once aimed; set each loop (true while shooting). */
    public AutoAim withLockWhenAimed(boolean lockWhenAimed) {
        this.lockWhenAimed = lockWhenAimed;
        return this;
    }

    public boolean isLocked() {
        return locked;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;

        this.pointToFace = FieldCalculations.aimPoint(currentPose);

        // The shooter is on the robot rear, so point the rear at the target.
        this.TargetDirection =
                pointToFace.minus(currentPose.getTranslation()).getAngle().rotateBy(Rotation2d.k180deg);
        double errorDeg = this.TargetDirection.minus(currentPose.getRotation()).getDegrees();
        updateLock(Math.abs(errorDeg), parameters);

        if (++applyCount % kDogLogDecimation == 0) {
            DogLog.log("AutoAim/PointToFace", pointToFace);
            DogLog.log("AutoAim/Error", Math.abs(errorDeg));
            DogLog.log("AutoAim/X Lock", locked);
        }
        headingTarget.append(this.TargetDirection.getDegrees());
        headingMeasured.append(currentPose.getRotation().getDegrees());
        headingError.append(errorDeg);
        headingLocked.update(locked);

        if (locked) {
            return xMode.apply(parameters, modulesToApply);
        }
        StatusCode status = super.apply(parameters, modulesToApply);
        headingOutput.append(HeadingController.getLastAppliedOutput());
        return status;
    }

    /** Hysteresis between kAimLockEnterDeg and kAimLockExitDeg keeps vision heading noise from toggling the lock. */
    private void updateLock(double absErrorDeg, SwerveControlParameters parameters) {
        boolean translating = Math.hypot(VelocityX, VelocityY) > kAimLockMaxTranslationMps;
        if (!lockWhenAimed || translating) {
            locked = false;
        } else if (locked) {
            locked = absErrorDeg <= kAimLockExitDeg;
        } else {
            locked = absErrorDeg < kAimLockEnterDeg
                    && Math.abs(parameters.currentChassisSpeed.omegaRadiansPerSecond) < kAimLockMaxTurnRateRadPerSec;
        }
    }
}
