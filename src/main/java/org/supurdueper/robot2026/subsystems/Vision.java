// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.lib.LimelightHelpers;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.Robot;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;

public class Vision extends SubsystemBase implements SupurdueperSubsystem {

    public static final String backLimelightName = "limelight-b";
    public static final String rightLimelightName = "limelight-r";
    public static final String leftLimelightName = "limelight-l";

    public Vision() {
        Robot.add(this);
    }

    @Override
    public void periodic() {
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        SwerveDriveState state = drivetrain.getState();
        updatePose3dAprilTag(backLimelightName, drivetrain, state);
        // updatePose3dAprilTag(rightLimelightName, drivetrain, state);
        // updatePose3dAprilTag(leftLimelightName, drivetrain, state);
    }

    private void updatePose3dAprilTag(String limelightName, Drivetrain drivetrain, SwerveDriveState state) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        boolean shouldUpdate = true;
        if (Math.abs(state.Speeds.omegaRadiansPerSecond) > Units.degreesToRadians(360)) {
            shouldUpdate = false;
        }
        if (mt2 == null) {
            shouldUpdate = false;
            return;
        }
        if (mt2.tagCount == 0) {
            shouldUpdate = false;
        }
        if (mt2.rawFiducials.length == 1) {
            double ambiguity = mt2.rawFiducials[0].ambiguity;
            if (ambiguity >= .7) {
                shouldUpdate = false;
            }
        }
        if (Math.abs(mt2.pose.getRotation().minus(state.Pose.getRotation()).getDegrees()) > 30) { // 1 meter
            shouldUpdate = false;
        }
        if (shouldUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        DogLog.log("Vision/" + limelightName + " Pose (mt2)", mt2.pose);
    }

    public static void updateIMUMode() {
        LimelightHelpers.SetIMUMode(rightLimelightName, 0);
        LimelightHelpers.SetIMUMode(backLimelightName, 0);
        LimelightHelpers.SetIMUMode(leftLimelightName, 0);
    }

    public static void setDisabled() {
        LimelightHelpers.SetThrottle(rightLimelightName, 150);
        LimelightHelpers.SetThrottle(backLimelightName, 150);
        LimelightHelpers.SetThrottle(leftLimelightName, 150);
    }

    public static void setEnabled() {
        LimelightHelpers.SetThrottle(rightLimelightName, 0);
        LimelightHelpers.SetThrottle(backLimelightName, 0);
        LimelightHelpers.SetThrottle(leftLimelightName, 0);
    }

    public static void setAprilTagFilter() {
        int[] ids = {2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25, 26, 27, 29, 30, 31, 32};
        LimelightHelpers.SetFiducialIDFiltersOverride(backLimelightName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(rightLimelightName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(leftLimelightName, ids);
    }

    @Override
    public void bindCommands() {}
}
