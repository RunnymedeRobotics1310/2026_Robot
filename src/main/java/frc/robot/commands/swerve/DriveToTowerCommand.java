package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToTowerCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 50; // TODO: fixme: move these to constants
  private static final int LEFT_TOWER_TX_OFFSET = -10;
  private static final int RIGHT_TOWER_TX_OFFSET = 29;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private final String VISION_PRIMARY_LIMELIGHT_NAME =
      Constants.VisionConstants.VISION_PRIMARY_LIMELIGHT_NAME;

  private int tagId = -1;
  private int noDataCount = 0;
  private final int tXOffset;
  private int theta = 0;

  public DriveToTowerCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, boolean isRightSide) {
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
    final double tA;
    if (vision.isTagInView(tagId, VISION_PRIMARY_LIMELIGHT_NAME)) {
      noDataCount = 0;
      tX = vision.angleToTarget(tagId, VISION_PRIMARY_LIMELIGHT_NAME);
      tA = vision.areaOfTarget(tagId, VISION_PRIMARY_LIMELIGHT_NAME);
    } else {
      noDataCount++;
      log("Tag " + tagId + " not in view");

      double omega = swerve.computeOmega(theta);

      // if more than 5º off, don't drive, just rotate
      if (Math.abs(swerve.getYaw() - theta) > 5) {
        swerve.driveRobotOriented(0, 0, omega);
        log("turning");
      } else {
        // TODO: Do we need this? - YES
        swerve.driveRobotOriented(0, -0.7, omega);
        log("driving");
      }
      return;
    }

    // drive to tag
    final double vX; // forward/backward speed
    final double vY; // left/right speed
    if (Math.abs(tX + tXOffset) > 10) {
      // if offset is big, don't go forwards, unless ur far away
      //      if (Math.abs(tA) < 0.5 ) { // Untested 1!!!1!1!!!11!!1
      vX = 0.4;
      // } else {
      //        vX = 0;
      // }
    } else {
      vX = 0.2;
    }

    // align to tag
    vY = -0.07 * (tX + tXOffset);
    log("tx: " + tX);

    double omega = swerve.computeOmega(theta);
    swerve.driveRobotOriented(vX, vY, omega);
  }

  @Override
  public boolean isFinished() {

    // if u can't see the tag for a few secs, stop
    if (noDataCount > MAX_NO_DATA_COUNT_CYCLES && Math.abs(swerve.getYaw() - theta) < 5) {
      log("Finishing - no vision data for " + noDataCount + " cycles");
      //      return true;
    }

    // if ur in the spot, stop
    final double tY = vision.heightOfTarget(tagId, VISION_PRIMARY_LIMELIGHT_NAME);
    log("TY: " + tY);
    //    return tY < -6.5; // tY when aligned is -6.7ish
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    noDataCount = 0;
    swerve.stop();
  }
}
