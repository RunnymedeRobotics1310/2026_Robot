package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.LinkedList;

/**
 * Comprehensive odometry debugging telemetry for comparing wheel odometry vs vision position.
 * Provides real-time and post-match analysis capabilities.
 */
public class OdometryDebugTelemetry {

  private static final String NT_PREFIX = PREFIX + "OdoDebug";

  /** Maximum number of poses to keep in history (5 seconds at 50Hz). */
  private static final int MAX_HISTORY_SIZE = 250;

  // Pose publishers for AdvantageScope compatibility
  private final StructPublisher<Pose2d> odometryPosePublisher;
  private final StructPublisher<Pose2d> visionPosePublisher;
  private final StructPublisher<Pose2d> visionSecondaryPosePublisher;
  private final StructPublisher<Pose2d> wheelOnlyPosePublisher;
  private final StructArrayPublisher<Pose2d> odometryTrailPublisher;
  private final StructArrayPublisher<Pose2d> visionTrailPublisher;
  private final StructArrayPublisher<Pose2d> wheelOnlyTrailPublisher;

  // Delta publishers (fused vs vision - legacy)
  private final DoublePublisher deltaDistancePublisher;
  private final DoublePublisher deltaHeadingPublisher;
  private final DoublePublisher deltaAccumulatedPublisher;
  private final DoublePublisher deltaAccumulatedHeadingPublisher;
  private final BooleanPublisher visionValidPublisher;
  private final DoublePublisher visionTimestampPublisher;

  // New delta publishers for three-way comparison
  private final DoublePublisher wheelVsVisionDistancePublisher;
  private final DoublePublisher wheelVsVisionHeadingPublisher;
  private final DoublePublisher wheelVsFusedDistancePublisher;
  private final DoublePublisher wheelVsFusedHeadingPublisher;

  // Slip publishers
  private final BooleanPublisher slipDetectedPublisher;
  private final DoublePublisher slipGyroVsWheelDeltaPublisher;
  private final DoublePublisher slipGyroRatePublisher;
  private final DoublePublisher slipWheelRatePublisher;

  // Pose history buffers (circular)
  private final LinkedList<TimestampedPose> odometryHistory = new LinkedList<>();
  private final LinkedList<TimestampedPose> visionHistory = new LinkedList<>();
  private final LinkedList<TimestampedPose> wheelOnlyHistory = new LinkedList<>();

  // Current poses
  private Pose2d currentOdometryPose = new Pose2d();
  private Pose2d currentVisionPose = new Pose2d();
  private Pose2d currentSecondaryVisionPose = new Pose2d();
  private Pose2d currentWheelOnlyPose = new Pose2d();
  private double visionTimestamp = 0;
  private boolean visionValid = false;

  // Drift tracking
  private Pose2d initialOdometryPose = null;
  private Pose2d initialVisionPose = null;
  private double accumulatedDriftMeters = 0.0;
  private double accumulatedHeadingDriftDeg = 0.0;

  // Per-module telemetry
  private ModuleTelemetry[] moduleTelemetry = new ModuleTelemetry[4];

  // Slip detection
  private final SlipDetector slipDetector = new SlipDetector();

  public OdometryDebugTelemetry() {
    var nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable(NT_PREFIX);

    // Create struct publishers for proper Pose2d logging (AdvantageScope compatible)
    odometryPosePublisher =
        nt.getStructTopic(NT_PREFIX + "/Odometry/Pose", Pose2d.struct).publish();
    visionPosePublisher = nt.getStructTopic(NT_PREFIX + "/Vision/Pose", Pose2d.struct).publish();
    visionSecondaryPosePublisher =
        nt.getStructTopic(NT_PREFIX + "/Vision/SecondaryPose", Pose2d.struct).publish();
    wheelOnlyPosePublisher =
        nt.getStructTopic(NT_PREFIX + "/WheelOnly/Pose", Pose2d.struct).publish();
    odometryTrailPublisher =
        nt.getStructArrayTopic(NT_PREFIX + "/Odometry/Trail", Pose2d.struct).publish();
    visionTrailPublisher =
        nt.getStructArrayTopic(NT_PREFIX + "/Vision/Trail", Pose2d.struct).publish();
    wheelOnlyTrailPublisher =
        nt.getStructArrayTopic(NT_PREFIX + "/WheelOnly/Trail", Pose2d.struct).publish();

    // Delta publishers (legacy: fused vs vision)
    NetworkTable deltaTable = table.getSubTable("Delta");
    deltaDistancePublisher = deltaTable.getDoubleTopic("fusedVsVision").publish();
    deltaHeadingPublisher = deltaTable.getDoubleTopic("fusedVsVisionHeading").publish();
    deltaAccumulatedPublisher = deltaTable.getDoubleTopic("accumulated").publish();
    deltaAccumulatedHeadingPublisher = deltaTable.getDoubleTopic("accumulatedHeading").publish();

    // New three-way comparison publishers
    wheelVsVisionDistancePublisher = deltaTable.getDoubleTopic("wheelVsVision").publish();
    wheelVsVisionHeadingPublisher = deltaTable.getDoubleTopic("wheelVsVisionHeading").publish();
    wheelVsFusedDistancePublisher = deltaTable.getDoubleTopic("wheelVsFused").publish();
    wheelVsFusedHeadingPublisher = deltaTable.getDoubleTopic("wheelVsFusedHeading").publish();

    // Vision status publishers
    NetworkTable visionTable = table.getSubTable("Vision");
    visionValidPublisher = visionTable.getBooleanTopic("valid").publish();
    visionTimestampPublisher = visionTable.getDoubleTopic("timestamp").publish();

    // Slip publishers
    NetworkTable slipTable = table.getSubTable("Slip");
    slipDetectedPublisher = slipTable.getBooleanTopic("detected").publish();
    slipGyroVsWheelDeltaPublisher = slipTable.getDoubleTopic("gyroVsWheelDelta").publish();
    slipGyroRatePublisher = slipTable.getDoubleTopic("gyroRate").publish();
    slipWheelRatePublisher = slipTable.getDoubleTopic("wheelRate").publish();

    // Initialize module telemetry with empty data
    for (int i = 0; i < 4; i++) {
      moduleTelemetry[i] = new ModuleTelemetry("unknown", 0, 0, 0, 0, 0, 0);
    }
  }

  /**
   * Update the current odometry pose.
   *
   * @param pose the current odometry pose
   */
  public void updateOdometryPose(Pose2d pose) {
    if (!Constants.TelemetryConfig.odometryDebugEnabled) {
      return;
    }

    currentOdometryPose = pose;
    double timestamp = Timer.getFPGATimestamp();

    // Initialize reference pose on first update
    if (initialOdometryPose == null) {
      initialOdometryPose = pose;
    }

    // Add to history
    odometryHistory.addLast(new TimestampedPose(pose, timestamp));
    while (odometryHistory.size() > MAX_HISTORY_SIZE) {
      odometryHistory.removeFirst();
    }

    // Update accumulated drift if we have vision
    updateDrift();
  }

  /**
   * Update the current vision pose.
   *
   * @param pose the vision-measured pose
   * @param timestamp the timestamp of the vision measurement
   * @param isValid whether the vision measurement is valid
   */
  public void updateVisionPose(
      Pose2d pose, Pose2d secondaryPose, double timestamp, boolean isValid) {
    if (!Constants.TelemetryConfig.odometryDebugEnabled) {
      return;
    }

    currentVisionPose = pose;
    currentSecondaryVisionPose = secondaryPose;
    visionTimestamp = timestamp;
    visionValid = isValid;

    if (isValid) {
      // Initialize reference pose on first valid update
      if (initialVisionPose == null) {
        initialVisionPose = pose;
      }

      // Add to history
      visionHistory.addLast(new TimestampedPose(pose, timestamp));
      while (visionHistory.size() > MAX_HISTORY_SIZE) {
        visionHistory.removeFirst();
      }
    }
  }

  /**
   * Update the current wheel-only pose. This is the pose estimated purely from wheel encoders and
   * gyro, without any vision corrections.
   *
   * @param pose the wheel-only odometry pose
   */
  public void updateWheelOnlyPose(Pose2d pose) {
    if (!Constants.TelemetryConfig.odometryDebugEnabled) {
      return;
    }

    currentWheelOnlyPose = pose;
    double timestamp = Timer.getFPGATimestamp();

    // Add to history
    wheelOnlyHistory.addLast(new TimestampedPose(pose, timestamp));
    while (wheelOnlyHistory.size() > MAX_HISTORY_SIZE) {
      wheelOnlyHistory.removeFirst();
    }
  }

  /**
   * Update per-module telemetry data.
   *
   * @param index module index (0-3)
   * @param telemetry the module telemetry data
   */
  public void updateModuleTelemetry(int index, ModuleTelemetry telemetry) {
    if (index >= 0 && index < 4) {
      moduleTelemetry[index] = telemetry;
    }
  }

  /**
   * Update slip detection with rotation rate data.
   *
   * @param gyroRotationRateRadPS gyro-measured rotation rate
   * @param wheelRotationRateRadPS wheel-calculated rotation rate
   */
  public void updateSlipDetection(double gyroRotationRateRadPS, double wheelRotationRateRadPS) {
    if (!Constants.TelemetryConfig.odometryDebugEnabled) {
      return;
    }
    slipDetector.update(gyroRotationRateRadPS, wheelRotationRateRadPS);
  }

  /** Update drift calculations based on current odometry vs vision. */
  private void updateDrift() {
    if (!visionValid || initialOdometryPose == null || initialVisionPose == null) {
      return;
    }

    // Calculate instantaneous delta
    double distanceDelta =
        currentOdometryPose.getTranslation().getDistance(currentVisionPose.getTranslation());
    double headingDelta =
        currentOdometryPose.getRotation().getDegrees()
            - currentVisionPose.getRotation().getDegrees();

    // Normalize heading delta
    while (headingDelta > 180) headingDelta -= 360;
    while (headingDelta < -180) headingDelta += 360;

    // Track accumulated drift (max observed)
    accumulatedDriftMeters = Math.max(accumulatedDriftMeters, distanceDelta);
    accumulatedHeadingDriftDeg = Math.max(accumulatedHeadingDriftDeg, Math.abs(headingDelta));
  }

  /** Reset drift tracking. Call this when resetting odometry. */
  public void resetDrift() {
    initialOdometryPose = null;
    initialVisionPose = null;
    accumulatedDriftMeters = 0.0;
    accumulatedHeadingDriftDeg = 0.0;
    odometryHistory.clear();
    visionHistory.clear();
    wheelOnlyHistory.clear();
    slipDetector.reset();
  }

  /** Post telemetry to NetworkTables. */
  void post() {
    if (!Constants.TelemetryConfig.odometryDebugEnabled) {
      return;
    }

    // Publish current poses (struct format for AdvantageScope)
    odometryPosePublisher.set(currentOdometryPose);
    visionPosePublisher.set(currentVisionPose);
    visionSecondaryPosePublisher.set(currentSecondaryVisionPose);
    wheelOnlyPosePublisher.set(currentWheelOnlyPose);

    // Publish trails
    Pose2d[] odoTrail = odometryHistory.stream().map(tp -> tp.pose).toArray(Pose2d[]::new);
    Pose2d[] visTrail = visionHistory.stream().map(tp -> tp.pose).toArray(Pose2d[]::new);
    Pose2d[] wheelTrail = wheelOnlyHistory.stream().map(tp -> tp.pose).toArray(Pose2d[]::new);
    odometryTrailPublisher.set(odoTrail);
    visionTrailPublisher.set(visTrail);
    wheelOnlyTrailPublisher.set(wheelTrail);

    visionValidPublisher.set(visionValid);
    visionTimestampPublisher.set(visionTimestamp);

    if (visionValid) {
      // Delta between fused odometry and vision
      deltaDistancePublisher.set(
          currentOdometryPose.getTranslation().getDistance(currentVisionPose.getTranslation()));
      deltaHeadingPublisher.set(
          normalizeHeading(
              currentOdometryPose.getRotation().getDegrees()
                  - currentVisionPose.getRotation().getDegrees()));
      deltaAccumulatedPublisher.set(accumulatedDriftMeters);
      deltaAccumulatedHeadingPublisher.set(accumulatedHeadingDriftDeg);

      // Three-way comparison: wheel-only vs vision (TRUE wheel error)
      wheelVsVisionDistancePublisher.set(
          currentWheelOnlyPose.getTranslation().getDistance(currentVisionPose.getTranslation()));
      wheelVsVisionHeadingPublisher.set(
          normalizeHeading(
              currentWheelOnlyPose.getRotation().getDegrees()
                  - currentVisionPose.getRotation().getDegrees()));
    } else {
      // Zero out vision-dependent deltas when vision has no valid pose
      deltaDistancePublisher.set(0);
      deltaHeadingPublisher.set(0);
      deltaAccumulatedPublisher.set(accumulatedDriftMeters);
      deltaAccumulatedHeadingPublisher.set(accumulatedHeadingDriftDeg);
      wheelVsVisionDistancePublisher.set(0);
      wheelVsVisionHeadingPublisher.set(0);
    }

    // Wheel-only vs fused (how much vision corrected) - always valid
    wheelVsFusedDistancePublisher.set(
        currentWheelOnlyPose.getTranslation().getDistance(currentOdometryPose.getTranslation()));
    wheelVsFusedHeadingPublisher.set(
        normalizeHeading(
            currentWheelOnlyPose.getRotation().getDegrees()
                - currentOdometryPose.getRotation().getDegrees()));

    // Slip detection
    slipDetectedPublisher.set(slipDetector.isSlipDetected());
    slipGyroVsWheelDeltaPublisher.set(slipDetector.getGyroVsWheelDelta());
    slipGyroRatePublisher.set(slipDetector.getGyroRotationRateRadPS());
    slipWheelRatePublisher.set(slipDetector.getWheelRotationRateRadPS());
  }

  /** Normalize heading delta to [-180, 180] range. */
  private static double normalizeHeading(double heading) {
    while (heading > 180) heading -= 360;
    while (heading < -180) heading += 360;
    return heading;
  }

  /** Timestamped pose for history tracking. */
  private record TimestampedPose(Pose2d pose, double timestamp) {}
}
