// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.supurdueper.lib.LimelightHelpers;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.Constants.VisionConstants;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;
import org.supurdueper.robot2026.utils.FieldConstants;

public class Vision extends SubsystemBase implements SupurdueperSubsystem {

    // MegaTag2 trusts the yaw it was sent: skip frames while spinning fast or solved with a stale yaw
    private static final double kMaxYawRateRadPerSec = Units.degreesToRadians(90);
    private static final double kMaxYawMismatchDeg = 4.0;

    // While disabled, seed the field heading from MegaTag1, which solves yaw from the tags alone
    private static final int kSeedMinTags = 2;
    private static final double kSeedMaxTagDistMeters = 4.0;
    private static final int kSeedFrames = 5;
    private static final double kSeedMaxSpreadDeg = 3.0;
    private static final double kSeedMinCorrectionDeg = 1.0;

    private final Map<String, Double> lastMt2Timestamps = new HashMap<>();
    private final Map<String, Double> lastMt1Timestamps = new HashMap<>();
    private final List<Rotation2d> seedHeadingErrors = new ArrayList<>();
    private final Alert cameraPoseAlert = new Alert(
            "Limelight robot-space pose is not rear-facing (LL Yaw should be about 180); vision is ignored",
            Alert.AlertType.kError);

    public Vision() {
        Robot.add(this);
    }

    @Override
    public void periodic() {
        // robot_orientation_set is published from the odometry thread; send it now rather than on NT's periodic update
        LimelightHelpers.Flush();
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        SwerveDriveState state = drivetrain.getState();
        for (String limelightName : VisionConstants.kLimelightNames) {
            updatePose3dAprilTag(limelightName, drivetrain, state);
        }
    }

    private void updatePose3dAprilTag(String limelightName, Drivetrain drivetrain, SwerveDriveState state) {
        String logPrefix = "Vision/" + limelightName;
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        DogLog.log(logPrefix + " Connected", mt2 != null);
        if (mt2 == null) {
            return;
        }
        // The camera faces the robot rear; a forward-facing pose (e.g. entered for the other robot front) would put
        // every estimate on the wrong side of the tags, and the heading seed would flip field-centric driving
        double cameraYawDeg = Units.radiansToDegrees(LimelightHelpers.getCameraPose3d_RobotSpace(limelightName)
                .getRotation()
                .getZ());
        boolean cameraPoseRearFacing = Math.abs(cameraYawDeg) > 90;
        cameraPoseAlert.set(!cameraPoseRearFacing);
        if (!cameraPoseRearFacing) {
            return;
        }
        if (DriverStation.isDisabled()) {
            seedHeadingFromMegaTag1(limelightName, drivetrain, state);
        } else {
            seedHeadingErrors.clear();
        }
        // Outputs repeat between Limelight frames (especially while throttled); use each frame once
        if (mt2.timestampSeconds == lastMt2Timestamps.getOrDefault(limelightName, -1.0)) {
            return;
        }
        lastMt2Timestamps.put(limelightName, mt2.timestampSeconds);
        DogLog.log(logPrefix + " Tag Count", mt2.tagCount);
        if (mt2.tagCount == 0) {
            return;
        }
        boolean shouldUpdate = true;
        if (Math.abs(state.Speeds.omegaRadiansPerSecond) > kMaxYawRateRadPerSec) {
            shouldUpdate = false;
        }
        if (mt2.rawFiducials.length == 1 && mt2.rawFiducials[0] != null && mt2.rawFiducials[0].ambiguity >= .7) {
            shouldUpdate = false;
        }
        if (!isOnField(mt2.pose)) {
            shouldUpdate = false;
        }
        double yawMismatchDeg = drivetrain
                .samplePoseAt(mt2.timestampSeconds)
                .map(pose -> Math.abs(
                        mt2.pose.getRotation().minus(pose.getRotation()).getDegrees()))
                .orElse(180.0);
        if (yawMismatchDeg > kMaxYawMismatchDeg) {
            shouldUpdate = false;
        }
        if (shouldUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        DogLog.log(logPrefix + " Pose (mt2)", mt2.pose);
        DogLog.log(logPrefix + " Accepted", shouldUpdate);
        DogLog.log(logPrefix + " Yaw Mismatch (Deg)", yawMismatchDeg);
        DogLog.log(logPrefix + " Avg Tag Dist (m)", mt2.avgTagDist);
    }

    private void seedHeadingFromMegaTag1(String limelightName, Drivetrain drivetrain, SwerveDriveState state) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null || mt1.timestampSeconds == lastMt1Timestamps.getOrDefault(limelightName, -1.0)) {
            return;
        }
        lastMt1Timestamps.put(limelightName, mt1.timestampSeconds);
        Optional<Pose2d> poseAtFrame = drivetrain.samplePoseAt(mt1.timestampSeconds);
        if (mt1.tagCount < kSeedMinTags || mt1.avgTagDist > kSeedMaxTagDistMeters || poseAtFrame.isEmpty()) {
            seedHeadingErrors.clear();
            return;
        }
        // Heading error at the frame time, so turning the robot between frames does not matter
        seedHeadingErrors.add(mt1.pose.getRotation().minus(poseAtFrame.get().getRotation()));
        if (seedHeadingErrors.size() > kSeedFrames) {
            seedHeadingErrors.remove(0);
        }
        if (seedHeadingErrors.size() < kSeedFrames) {
            return;
        }
        double cos = 0;
        double sin = 0;
        for (Rotation2d error : seedHeadingErrors) {
            cos += error.getCos();
            sin += error.getSin();
        }
        Rotation2d meanError = new Rotation2d(cos, sin);
        for (Rotation2d error : seedHeadingErrors) {
            if (Math.abs(error.minus(meanError).getDegrees()) > kSeedMaxSpreadDeg) {
                return;
            }
        }
        DogLog.log("Vision/" + limelightName + " MT1 Heading Error (Deg)", meanError.getDegrees());
        if (Math.abs(meanError.getDegrees()) > kSeedMinCorrectionDeg) {
            drivetrain.resetRotation(state.Pose.getRotation().plus(meanError));
            seedHeadingErrors.clear();
        }
    }

    private static boolean isOnField(Pose2d pose) {
        return pose.getX() > 0
                && pose.getX() < FieldConstants.fieldLength
                && pose.getY() > 0
                && pose.getY() < FieldConstants.fieldWidth;
    }

    public static void updateIMUMode() {
        VisionConstants.kLimelightNames.forEach(name -> LimelightHelpers.SetIMUMode(name, 0));
    }

    /** The pose constants describe the single shooter-facing Limelight. */
    public static void setCameraPose() {
        VisionConstants.kLimelightNames.forEach(name -> LimelightHelpers.setCameraPose_RobotSpace(
                name,
                VisionConstants.kCameraForwardMeters,
                VisionConstants.kCameraRightMeters,
                VisionConstants.kCameraUpMeters,
                VisionConstants.kCameraRollDegrees,
                VisionConstants.kCameraPitchDegrees,
                VisionConstants.kCameraYawDegrees));
    }

    public static void setDisabled() {
        // Throttled to limit heat while disabled, but fast enough for the MegaTag1 heading seed
        VisionConstants.kLimelightNames.forEach(name -> LimelightHelpers.SetThrottle(name, 50));
    }

    public static void setEnabled() {
        VisionConstants.kLimelightNames.forEach(name -> LimelightHelpers.SetThrottle(name, 0));
    }

    public static void setAprilTagFilter() {
        int[] ids = {2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25, 26, 27, 29, 30, 31, 32};
        VisionConstants.kLimelightNames.forEach(name -> LimelightHelpers.SetFiducialIDFiltersOverride(name, ids));
    }

    @Override
    public void bindCommands() {}
}
