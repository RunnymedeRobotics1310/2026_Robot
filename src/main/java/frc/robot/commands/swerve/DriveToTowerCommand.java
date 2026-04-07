package frc.robot.commands.swerve;

import static frc.robot.Constants.VisionConstants.VISION_SECONDARY_LIMELIGHT_NAME;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
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
  private static final int LEFT_TOWER_TX_OFFSET = 24; // robot left
  private static final int RIGHT_TOWER_TX_OFFSET = -13; // robot right

  // 32.75in -> between posts
  // 147.47in +- (32.75in/2)
  // 163.845in = 416.17cm -> left
  // 131.095in = 332.98cm -> right
  private static final double BLUE_TOWER_LEFT_POST_M = 4.1617;
  private static final double BLUE_TOWER_RIGHT_POST_M = 3.3298;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private int tagId = -1;
  private int noDataCount = 0;
  private final int tXOffset;
  private final double yCoord;
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
      if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
        yCoord = BLUE_TOWER_RIGHT_POST_M;
      } else {
        yCoord = Constants.FieldConstants.FIELD_EXTENT_METRES_Y - BLUE_TOWER_RIGHT_POST_M;
      }
    } else {
      tXOffset = LEFT_TOWER_TX_OFFSET;
      if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
        yCoord = BLUE_TOWER_LEFT_POST_M;
      } else {
        yCoord = Constants.FieldConstants.FIELD_EXTENT_METRES_Y - BLUE_TOWER_LEFT_POST_M;
      }
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

    // drive to tag
    double vX; // forward/backward speed
    final double vY; // left/right speed

    final double y = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseY();
    final double x = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseX();
    final double wallDist;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
      wallDist = x;
    } else {
      wallDist = Constants.FieldConstants.FIELD_EXTENT_METRES_X - x;
    }
    final double errorM = yCoord - y;

    vY = -2 * errorM;

    if (wallDist > 1.8) vX = 1; // go faster if further away from thing
    else vX = .25; // slow zone for last 15cm

    if (wallDist < 1.7 && Math.abs(errorM) > .05) vX = 0; // too close and not aligned
    else if (Math.abs(errorM) > .5) vX = 0; // too far and not aligned

    double omega = swerve.computeOmega(theta);
    if (SwerveUtils.isCloseEnough(swerve.getYaw(), theta, 5))
      swerve.driveRobotOriented(vX, vY, omega);
    else swerve.driveRobotOriented(0, 0, omega);
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
    //    log("TY: " + tY);

    if (DriverStation.isAutonomous() && DriverStation.getMatchTime() <= 2) return true;

    final double y = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseY();
    final double x = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseX();
    boolean yAligned = Math.abs(yCoord - y) < .02;
    boolean xAligned = x < 1.6;

    return yAligned && xAligned;

    //    return tY < -11.4; // tY when aligned is 11.5ish
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
