package org.supurdueper.robot2026.subsystems.drive.generated;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.List;
import org.supurdueper.robot2026.Constants.VisionConstants;
import org.supurdueper.robot2026.utils.FieldCalculations;

/**
 * Swerve state telemetry from the 250 Hz odometry thread. Only the Limelight robot orientation (MegaTag2) is published
 * every state; NT DriveState and DogLog get every 5th state (50 Hz), the tuning arrays every 2nd (125 Hz). The CTRE
 * template's per-module Mechanism2d widgets were removed: about 2000 NT writes per second on a CPU-saturated rio.
 */
public class DriveTelemetry {

    /**
     * Construct a telemetry object
     *
     * @param maxSpeed Maximum speed in meters per second (unused since the module Mechanism2d widgets were removed)
     */
    public DriveTelemetry(double maxSpeed) {
        fieldTypePub.set("Field2d");
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final List<DoubleArrayPublisher> limelightRobotOrientationPublishers =
            VisionConstants.kLimelightNames.stream()
                    .map(name -> inst.getTable(name)
                            .getDoubleArrayTopic("robot_orientation_set")
                            .publish())
                    .toList();
    private double[] robotOrientation = {0, 0, 0, 0, 0, 0};

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose =
            driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds =
            driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
            .publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish();
    private final DoublePublisher driveTimestamp =
            driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency =
            driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub =
            table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final double[] m_poseArray = new double[3];

    // Every 5th state (50 Hz): NT DriveState and DogLog. DogLog every state overflowed its queue (MAX_QUEUED_LOGS on
    // DriveState/ModuleStates).
    private static final int kDecimation = 5;
    private int telemeterizeCount = 0;

    // Tuning telemetry (DataLog, not DogLog): module speeds and angles at 125 Hz (every 2nd state) for the drive and
    // steer analysis, module order FL, FR, BL, BR
    private static final int kTuningDecimation = 2;
    private final DoubleArrayLogEntry steerAnglesEntry =
            new DoubleArrayLogEntry(DataLogManager.getLog(), "/Tuning/SwerveSteer/Angles");
    private final DoubleArrayLogEntry steerTargetsEntry =
            new DoubleArrayLogEntry(DataLogManager.getLog(), "/Tuning/SwerveSteer/Targets");
    private final DoubleArrayLogEntry driveSpeedsEntry =
            new DoubleArrayLogEntry(DataLogManager.getLog(), "/Tuning/SwerveDrive/Speeds");
    private final DoubleArrayLogEntry driveTargetSpeedsEntry =
            new DoubleArrayLogEntry(DataLogManager.getLog(), "/Tuning/SwerveDrive/TargetSpeeds");
    private final double[] moduleAngles = new double[4];
    private final double[] moduleTargetAngles = new double[4];
    private final double[] moduleSpeeds = new double[4];
    private final double[] moduleTargetSpeeds = new double[4];

    /** Accept the swerve drive state and telemeterize it to NetworkTables and the logs. */
    public void telemeterize(SwerveDriveState state) {
        telemeterizeCount++;

        // MegaTag2 needs the latest heading every state
        robotOrientation[0] = state.Pose.getRotation().getDegrees();
        for (DoubleArrayPublisher publisher : limelightRobotOrientationPublishers) {
            publisher.set(robotOrientation);
        }

        /* Tuning telemetry (CTRE SignalLogger is off; see Robot.robotInit) */
        if (telemeterizeCount % kTuningDecimation == 0) {
            // Rounded (0.01 deg, 1 mm/s) and written only on change, so a stationary robot adds no records
            for (int i = 0; i < 4; ++i) {
                moduleAngles[i] = Math.round(state.ModuleStates[i].angle.getDegrees() * 100) / 100.0;
                moduleTargetAngles[i] = Math.round(state.ModuleTargets[i].angle.getDegrees() * 100) / 100.0;
                moduleSpeeds[i] = Math.round(state.ModuleStates[i].speedMetersPerSecond * 1000) / 1000.0;
                moduleTargetSpeeds[i] = Math.round(state.ModuleTargets[i].speedMetersPerSecond * 1000) / 1000.0;
            }
            steerAnglesEntry.update(moduleAngles);
            steerTargetsEntry.update(moduleTargetAngles);
            driveSpeedsEntry.update(moduleSpeeds);
            driveTargetSpeedsEntry.update(moduleTargetSpeeds);
        }

        if (telemeterizeCount % kDecimation != 0) {
            return;
        }

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Telemeterize the pose to a Field2d */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Log to DogLog */
        DogLog.log("DriveState/Speeds", state.Speeds);
        DogLog.log("DriveState/ModuleStates", state.ModuleStates);
        DogLog.log("DriveState/ModuleTargets", state.ModuleTargets);
        DogLog.log("DriveState/ModulePositions", state.ModulePositions);
        DogLog.log("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
        DogLog.log("Pose", m_poseArray);
        DogLog.log(
                "Distance To Goal", FieldCalculations.distanceToGoal(state.Pose).in(Meters));
        DogLog.log("Our Zone", FieldCalculations.ourZone(state.Pose));
    }
}
