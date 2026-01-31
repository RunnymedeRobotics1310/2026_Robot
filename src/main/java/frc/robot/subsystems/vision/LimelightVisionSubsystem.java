package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

import static frc.robot.Constants.VisionConstants.*;

public class LimelightVisionSubsystem extends SubsystemBase {

  // MegaTags
  private final DoubleArraySubscriber hughMegaTag;

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose hughBotPoseCache = new LimelightBotPose();

  private final SwerveSubsystem swerve;

  public LimelightVisionSubsystem(VisionConfig visionConfig, SwerveSubsystem swerve) {
    this.swerve = swerve;

    Telemetry.vision.telemetryLevel = visionConfig.telemetryLevel();

    final NetworkTable hugh =
        NetworkTableInstance.getDefault().getTable("limelight-" + VISION_PRIMARY_LIMELIGHT_NAME);

    // Initialize the NT subscribers for whichever of MT1/2 is used
    hughMegaTag = hugh.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

    // inputs/configs
    hugh.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    hugh.getEntry("camMode").setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    // Pull data from the limelights and update our cache
    hughBotPoseCache.update(hughMegaTag.getAtomic());

    // Update telemetry
    updateTelemetry();
  }


  /**
   * If targeting the left reef, use the left reef pose, otherwise use the right reef pose
   *
   * @return Appropriate botPose data for Hugh
   */
  private LimelightBotPose getBotPose() {
    return hughBotPoseCache;
  }

  /* Public API */

  /**
   * Get the tag ID of the closest visible target to the limelight handling left or right branch
   *
   * @return the tag ID of the closest visible target to the limelight handling left or right branch
   */
  public double getVisibleTargetTagId() {
    return getBotPose().getTagId(0);
  }

  /**
   * Get the number of tags visible to the default limelight (hugh)
   *
   * @return the number of tags visible to the default limelight (hugh)
   */
  public int getNumTagsVisible() {
    return (int) hughBotPoseCache.getTagCount();
  }

  /**
   * Obtain the distance to robot centre to the tag either nearest to, or targeted if one has been
   * set by setTargetTag(), to the default limelight (hugh)
   *
   * @return the distance to robot centre to the nearest or targeted tag
   */
  public double distanceTagToRobot() {
    return distanceTagToRobot(0, true);
  }

  /**
   * Obtain the distance to robot centre to the tag either nearest to, or targeted if one has been
   * set by setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the distance to robot centre to the nearest or targeted tag
   */
  public double distanceTagToRobot(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose();

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToRobot(index);
  }

  /**
   * Obtain the distance to the camera for tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the default limelight (hugh)
   *
   * @return the distance to camera to the nearest or targeted tag
   */
  public double distanceTagToCamera() {
    return distanceTagToCamera(0, true);
  }

  /**
   * Obtain the distance to camera to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the distance to camera to the nearest or targeted tag
   */
  public double distanceTagToCamera(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose();

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToCamera(index);
  }

  /**
   * Obtain the angle to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the default limelight (hugh)
   *
   * @return the angle to the nearest or targeted tag
   */
  public double angleToTarget() {
    return angleToTarget(0, true);
  }

  /**
   * Obtain the angle to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the angle to the nearest or targeted tag
   */
  public double angleToTarget(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose();

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return -botPose.getTagTxnc(index);
  }

  /**
   * Get the number of tags visible to the default limelight (hugh)
   *
   * @return the number of tags visible to the default limelight (hugh)
   */
  public double getTagCount() {
    return hughBotPoseCache.getTagCount();
  }

  /**
   * Checks if a specific tag is visible to the default limelight (hugh)
   *
   * @param tagId The ID of the tag to check
   * @return If tagId is visible or not
   */
  public boolean isTagInView(int tagId) {
    return isTagInView(tagId, true);
  }

  /**
   * Checks if a specific tag is visible to the limelight handling left or right branches.
   *
   * @param tagId The ID of the tag to check
   * @return If tagId is visible or not
   */
  public boolean isTagInView(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose();
    return botPose.getTagIndex(tagId) != -1;
  }

  /** Update telemetry with vision data */
  private void updateTelemetry() {
    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.REGULAR
        || Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {

      Pose2d odometryPose = swerve.getPose();
      double yaw = swerve.getYaw();

      double compareDistance =
          hughBotPoseCache.getPose().getTranslation().getDistance(odometryPose.getTranslation());
      double compareHeading =
          hughBotPoseCache.getPose().getRotation().getDegrees()
              - odometryPose.getRotation().getDegrees();

      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = hughBotPoseCache.getPoseX();
      Telemetry.vision.visionPoseY = hughBotPoseCache.getPoseY();
      Telemetry.vision.visionPoseHeading = hughBotPoseCache.getPoseRotationYaw();
      Telemetry.vision.navxYaw = yaw;
      Telemetry.vision.navxYawDelta = odometryPose.getRotation().getDegrees() - yaw;
    }

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      Telemetry.vision.poseXSeries.add(hughBotPoseCache.getPoseX());
      Telemetry.vision.poseYSeries.add(hughBotPoseCache.getPoseY());
      Telemetry.vision.poseDegSeries.add(hughBotPoseCache.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = hughBotPoseCache.getVisibleTags();
      Telemetry.vision.nikTx = hughBotPoseCache.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = hughBotPoseCache.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = hughBotPoseCache.getTagDistToCamera(0);

    }
  }

  @Override
  public String toString() {
    return "AC/DeepSea Vision Subsystem";
  }
}
