// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.supurdueper.robot2026.utils;

import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.lib.utils.GeomUtil;
import org.supurdueper.robot2026.Constants;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  @Getter private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}
  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    phaseDelay = 0.03;

    hoodAngleMap.put(0.96, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(1.16, Rotation2d.fromDegrees(12.0));
    hoodAngleMap.put(1.58, Rotation2d.fromDegrees(14.0));
    hoodAngleMap.put(2.07, Rotation2d.fromDegrees(18.5));
    hoodAngleMap.put(2.37, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(2.47, Rotation2d.fromDegrees(23.0));
    hoodAngleMap.put(2.70, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.94, Rotation2d.fromDegrees(25.0));
    hoodAngleMap.put(3.48, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.92, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(4.35, Rotation2d.fromDegrees(34.0));
    hoodAngleMap.put(4.84, Rotation2d.fromDegrees(38.0));

    flywheelSpeedMap.put(0.96, 150.0);
    flywheelSpeedMap.put(1.16, 155.0);
    flywheelSpeedMap.put(1.58, 160.0);
    flywheelSpeedMap.put(2.07, 165.0);
    flywheelSpeedMap.put(2.37, 170.0);
    flywheelSpeedMap.put(2.47, 170.0);
    flywheelSpeedMap.put(2.70, 170.0);
    flywheelSpeedMap.put(2.94, 175.0);
    flywheelSpeedMap.put(3.48, 175.0);
    flywheelSpeedMap.put(3.92, 180.0);
    flywheelSpeedMap.put(4.35, 185.0);
    flywheelSpeedMap.put(4.84, 190.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(17.16, Rotation2d.fromDegrees(38.0));

    passingFlywheelSpeedMap.put(5.46, 160.0);
    passingFlywheelSpeedMap.put(6.62, 180.0);
    passingFlywheelSpeedMap.put(7.80, 200.0);
    passingFlywheelSpeedMap.put(17.16, 360.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.16);

  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    boolean passing =
        AllianceFlip.applyX(robotPose.getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = robotPose;
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double launcherToTargetDistance = target.getDistance(estimatedPose.getTranslation());

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = robotPose;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = robotVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = robotVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              robotPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              robotPose.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose;
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    boolean outsideOfBadBoxes = true;

    double flywheelVelocity = flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >=  minDistance
                && lookaheadLauncherToTargetDistance
                    <= maxDistance,
            driveAngle,
            driveVelocity,
            hoodAngle + Units.degreesToRadians(hoodAngleOffsetDeg),
            hoodVelocity,
            flywheelVelocity,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    DogLog.log("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    DogLog.log("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    DogLog.log(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    return latestParameters;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
    return target.minus(robotPose.getTranslation()).getAngle();
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }


  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlip.apply(target);
    }

    return new Pose2d(
        robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }
}