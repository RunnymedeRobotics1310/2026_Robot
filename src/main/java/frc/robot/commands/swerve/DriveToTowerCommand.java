package frc.robot.commands.swerve;

import static frc.robot.Constants.ClimbConstants.MAX_CLIMB_POSITION;
import static frc.robot.Constants.VisionConstants.VISION_SECONDARY_LIMELIGHT_NAME;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

@AutoConfigurable(
    value = "vision_approach_tag",
    category = "drive",
    description = "Drive toward an AprilTag using vision")
public class DriveToTowerCommand extends LoggingCommand {

  // 32.75in -> between posts
  // 147.47in +- (32.75in/2)
  // 163.845in = 416.17cm -> left
  // 131.095in = 332.98cm -> right
  private static final double BLUE_TOWER_LEFT_POST_M = 4.1617;
  private static final double BLUE_TOWER_RIGHT_POST_M = 3.3298;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private final ClimbSubsystem climb;
  private final boolean isRightSide;

  private double yCoord;
  private int theta = 0;

  public DriveToTowerCommand(
      SwerveSubsystem swerve,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      @ConfigParam(value = "rightSide", description = "Approach right side of tower")
          boolean isRightSide) {
    super();
    this.swerve = swerve;
    this.vision = vision;
    this.climb = climb;
    this.isRightSide = isRightSide;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    logCommandStart();
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
      theta = 180;
    }

    if (isRightSide) {
      if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
        yCoord = BLUE_TOWER_RIGHT_POST_M;
      } else {
        yCoord = Constants.FieldConstants.FIELD_EXTENT_METRES_Y - BLUE_TOWER_RIGHT_POST_M;
      }
    } else {
      if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
        yCoord = BLUE_TOWER_LEFT_POST_M;
      } else {
        yCoord = Constants.FieldConstants.FIELD_EXTENT_METRES_Y - BLUE_TOWER_LEFT_POST_M;
      }
    }
  }

  @Override
  public void execute() {

    final double y = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseY();
    final double x = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseX();
    final double wallDist;
    final double errorM;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
      wallDist = x;
      errorM = y - yCoord;
    } else {
      wallDist = Constants.FieldConstants.FIELD_EXTENT_METRES_X - x;
      errorM = yCoord - y;
    }

    // drive to tag
    double vX = 0.75; // forward/backward speed
    double vY = 3 * errorM; // left/right speed

    if (wallDist < 1.75
        && (Math.abs(errorM) > .02 || climb.getClimbPosition() < MAX_CLIMB_POSITION))
      vX = 0; // too close and not aligned || too close and climb not up
    else if (Math.abs(errorM) > 1) vX = 0; // too far and not aligned
    else if (wallDist < 1.85) vX = .25; // go slower if close to the thing

    double omega = swerve.computeOmega(theta);
    if (SwerveUtils.isCloseEnough(swerve.getYaw(), theta, 5))
      swerve.driveRobotOriented(vX, vY, omega);
    else swerve.driveRobotOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {

    if (DriverStation.isAutonomous() && DriverStation.getMatchTime() <= 2) return true;

    final double y = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseY();
    final double x = vision.getBotPose(VISION_SECONDARY_LIMELIGHT_NAME).getPoseX();
    final double wallDist;
    final double errorM;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
      wallDist = x;
      errorM = y - yCoord;
    } else {
      wallDist = Constants.FieldConstants.FIELD_EXTENT_METRES_X - x;
      errorM = yCoord - y;
    }

    boolean yAligned = Math.abs(errorM) < .02;
    boolean xAligned = wallDist < 1.6;

    return yAligned && xAligned;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    yCoord = 0;
    theta = 0;
    swerve.stop();
  }
}
