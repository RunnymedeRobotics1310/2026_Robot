package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public class LimelightVisionSubsystem extends SubsystemBase {

  // MegaTags
  private final DoubleArraySubscriber primaryMegaTag;
  private final DoubleArraySubscriber secondaryMegaTag;
  private final DoubleArrayPublisher llRobotOrientation;

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose primaryLimelightPoseCache = new LimelightBotPose();
  private final LimelightBotPose secondaryLimelightPoseCache = new LimelightBotPose();

  private final SwerveSubsystem swerve;

  public LimelightVisionSubsystem(VisionConfig visionConfig, SwerveSubsystem swerve) {
    this.swerve = swerve;

    final NetworkTable llNT =
        NetworkTableInstance.getDefault().getTable("limelight-" + VISION_SECONDARY_LIMELIGHT_NAME);

    llRobotOrientation = llNT.getDoubleArrayTopic("robot_orientation_set").publish();

    Telemetry.vision.telemetryLevel = visionConfig.telemetryLevel();

    final NetworkTable primary =
        NetworkTableInstance.getDefault().getTable("limelight-" + VISION_PRIMARY_LIMELIGHT_NAME);
    final NetworkTable secondary =
        NetworkTableInstance.getDefault().getTable("limelight-" + VISION_SECONDARY_LIMELIGHT_NAME);

    // Initialize the NT subscribers for whichever of MT1/2 is used
    primaryMegaTag = primary.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    secondaryMegaTag =
        secondary.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

    // inputs/configs
    primary.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    primary.getEntry("camMode").setNumber(visionConfig.camModeVision());

    secondary.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    secondary.getEntry("camMode").setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    // update the secondary limelight heading
    llRobotOrientation.set(new double[] {swerve.getYaw(), 0, 0, 0, 0, 0});
    // Pull data from the limelights and update our cache
    primaryLimelightPoseCache.update(primaryMegaTag.getAtomic());
    secondaryLimelightPoseCache.update(secondaryMegaTag.getAtomic());

    // Update swerve subsystem with vision pose for Field2d and odometry debugging
    swerve.updateVisionPose(
        primaryLimelightPoseCache.getPose(),
        secondaryLimelightPoseCache.getPose(),
        primaryLimelightPoseCache.getTimestampSeconds(),
        primaryLimelightPoseCache.isPoseValid());

    // Update telemetry
    updateTelemetry();
  }

  /**
   * Get the botpose of the corresponding limelight
   *
   * @return Appropriate botPose data
   */
  private LimelightBotPose getBotPose(String limelightName) {
    if (limelightName.equals(VISION_PRIMARY_LIMELIGHT_NAME)) {
      return primaryLimelightPoseCache;
    } else if (limelightName.equals(VISION_SECONDARY_LIMELIGHT_NAME)) {
      return secondaryLimelightPoseCache;
    }
    return null; // Should this return null? Is there a better way to handle this case?
  }

  /* Public API */

  /**
   * Get the tag ID of the closest visible target to the limelight handling left or right branch
   *
   * @return the tag ID of the closest visible target to the limelight handling left or right branch
   */
  public double getVisibleTargetTagId(String limelightName) {
    return getBotPose(limelightName).getTagId(0);
  }

  /**
   * Get the number of tags visible to the default limelight
   *
   * @return the number of tags visible to the default limelight
   */
  public int getNumTagsVisible() {
    return (int) primaryLimelightPoseCache.getTagCount();
  }

  /**
   * Obtain the distance to robot centre to the tag either nearest to, or targeted if one has been
   * set by setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @return the distance to robot centre to the nearest or targeted tag
   */
  public double distanceTagToRobot(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToRobot(index);
  }

  /**
   * Obtain the distance to camera to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @return the distance to camera to the nearest or targeted tag
   */
  public double distanceTagToCamera(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToCamera(index);
  }

  /**
   * Obtain the angle to the tag either nearest to, or targeted if one has been set by
   * setTargetTag()
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @return the angle to the nearest or targeted tag
   */
  public double angleToTarget(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagTxnc(index);
  }

  public double heightOfTarget(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return -botPose.getTagTync(index);
  }

  public double areaOfTarget(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return -botPose.getTagTa(index);
  }

  /**
   * Get the number of tags visible to the default limelight
   *
   * @return the number of tags visible to the default limelight
   */
  public double getTagCount() {
    return primaryLimelightPoseCache.getTagCount();
  }

  /**
   * Get the latest vision pose with validity information.
   *
   * @return the vision pose from the primary limelight (nikola)
   */
  public Pose2d getVisionPose() {
    return primaryLimelightPoseCache.getPose();
  }

  /**
   * Check if the current vision pose is valid.
   *
   * @return true if the vision pose is valid and can be used for odometry correction
   */
  public boolean isVisionPoseValid() {
    return primaryLimelightPoseCache.isPoseValid();
  }

  /**
   * Checks if a specific tag is visible
   *
   * @param tagId The ID of the tag to check
   * @param limelightName the limelight you want to check
   * @return If tagId is visible or not
   */
  public boolean isTagInView(int tagId, String limelightName) {
    LimelightBotPose botPose = getBotPose(limelightName);
    return botPose.getTagIndex(tagId) != -1;
  }

  /** Update telemetry with vision data */
  private void updateTelemetry() {
    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.REGULAR
        || Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {

      Pose2d odometryPose = swerve.getPose();
      double yaw = swerve.getYaw();

      double compareDistance =
          primaryLimelightPoseCache
              .getPose()
              .getTranslation()
              .getDistance(odometryPose.getTranslation());
      double compareHeading =
          primaryLimelightPoseCache.getPose().getRotation().getDegrees()
              - odometryPose.getRotation().getDegrees();

      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = primaryLimelightPoseCache.getPoseX();
      Telemetry.vision.visionPoseY = primaryLimelightPoseCache.getPoseY();
      Telemetry.vision.visionPoseHeading = primaryLimelightPoseCache.getPoseRotationYaw();
      Telemetry.vision.navxYaw = yaw;
      Telemetry.vision.navxYawDelta = odometryPose.getRotation().getDegrees() - yaw;
    }

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      Telemetry.vision.poseXSeries.add(primaryLimelightPoseCache.getPoseX());
      Telemetry.vision.poseYSeries.add(primaryLimelightPoseCache.getPoseY());
      Telemetry.vision.poseDegSeries.add(primaryLimelightPoseCache.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = primaryLimelightPoseCache.getVisibleTags();
      Telemetry.vision.nikTx = primaryLimelightPoseCache.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = primaryLimelightPoseCache.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = primaryLimelightPoseCache.getTagDistToCamera(0);

      Telemetry.vision.tomVisibleTags = secondaryLimelightPoseCache.getVisibleTags();
      Telemetry.vision.tomTx = secondaryLimelightPoseCache.getTagTxnc(0);
      Telemetry.vision.tomDistanceToRobot = secondaryLimelightPoseCache.getTagDistToRobot(0);
      Telemetry.vision.tomDistanceToCam = secondaryLimelightPoseCache.getTagDistToCamera(0);
    }
  }

  @Override
  public String toString() {
    return "Swervy Vision Subsystem";
  }
}
