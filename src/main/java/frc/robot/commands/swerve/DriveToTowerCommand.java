package frc.robot.commands.swerve;

import static frc.robot.Constants.VisionConstants.VISION_SECONDARY_LIMELIGHT_NAME;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

@AutoConfigurable(
    value = "vision_approach_tag",
    category = "drive",
    description = "Drive toward an AprilTag using vision")
public class DriveToTowerCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 50; // TODO: fixme: move these to constants
  private static final int LEFT_TOWER_TX_OFFSET = 23; // robot left
  private static final int RIGHT_TOWER_TX_OFFSET = -13; // robot right

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private int tagId = -1;
  private int noDataCount = 0;
  private final int tXOffset;
  private int theta = 0;

  public DriveToTowerCommand(
      SwerveSubsystem swerve,
      LimelightVisionSubsystem vision,
      @ConfigParam(value = "rightSide", description = "Approach right side of tower")
          boolean isRightSide) {
    super();
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);

    if (isRightSide) {
      tXOffset = RIGHT_TOWER_TX_OFFSET;
    } else {
      tXOffset = LEFT_TOWER_TX_OFFSET;
    }
  }

  @Override
  public void initialize() {
    logCommandStart();
    noDataCount = 0;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      tagId = 15;
    } else {
      tagId = 31;
      theta = 180;
    }
  }

  @Override
  public void execute() {

    // get offset
    final double tX;
    if (vision.isTagInView(tagId, VISION_SECONDARY_LIMELIGHT_NAME)) {
      noDataCount = 0;
      tX = vision.angleToTarget(tagId, VISION_SECONDARY_LIMELIGHT_NAME);
    } else {
      noDataCount++;
      log("Tag " + tagId + " not in view");

      double omega = swerve.computeOmega(theta);
      // if more than 5º off, don't drive, just rotate
      double normalYaw = (swerve.getYaw() + 360) % 360;
      if (Math.abs(normalYaw - theta) > 5) {
        swerve.driveRobotOriented(0, 0, omega);
      } else {
        swerve.driveRobotOriented(0, -0.7, omega);
      }
      return;
    }

    // drive to tag
    final double vX; // forward/backward speed
    final double vY; // left/right speed
    if (Math.abs(tX + tXOffset) > 10) {
      vX = 0;
    } else {
      vX = 0.4;
    }

    // align to tag
    vY = -0.1 * (tX + tXOffset);

    double omega = swerve.computeOmega(theta);
    swerve.driveRobotOriented(vX, vY, omega);
  }

  @Override
  public boolean isFinished() {

    // if u can't see the tag for a few secs, stop
    if (noDataCount > MAX_NO_DATA_COUNT_CYCLES && Math.abs(swerve.getYaw() - theta) < 5) {
      log("Finishing - no vision data for " + noDataCount + " cycles");
      return true;
    }

    // if ur in the spot, stop
    final double tY = vision.heightOfTarget(tagId, VISION_SECONDARY_LIMELIGHT_NAME);
    log("TY: " + tY);
    return tY < -11.6; // tY when aligned is 6.7ish
    //      return false;
    // will eventually end based on a sensor in the climb
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    noDataCount = 0;
    swerve.stop();
  }
}
