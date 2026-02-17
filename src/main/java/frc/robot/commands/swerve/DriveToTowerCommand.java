package frc.robot.commands.swerve;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToTowerCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 50;
  private static final int TOWER_TX_OFFSET = 20;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private int tagId = -1;
  private int noDataCount = 0;
  private int theta = 0;


  public DriveToTowerCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, boolean rightSide) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    logCommandStart();

    noDataCount = 0;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      tagId = 15;
      theta = 180; // this will change when we get the comp bot
    } else {
      tagId = 31;
    }
  }

  @Override
  public void execute() {

    // get offset
    final double tX;
    if (vision.isTagInView(tagId)) {
      noDataCount = 0;
      tX = vision.angleToTarget(tagId);
    } else {
      noDataCount++;

      double omega = swerve.computeOmega(theta);

      if (Math.abs(swerve.getYaw() - theta) > 5) {
        swerve.driveRobotOriented(0, 0, omega);
      } else {
        swerve.driveRobotOriented(0, -0.7, omega);
      }
      return;
    }

    // drive to tag
    final double vX;
    final double vY;
    if (Math.abs(tX+TOWER_TX_OFFSET) > 20) {
      vX = 0;
    } else {
      vX = -0.35;
    }
    vY = 0.07 * (tX+TOWER_TX_OFFSET);

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
    final double tY = vision.heightOfTarget(tagId);
    log("TY: " + tY);
    return tY > -11 && tY < -1;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    noDataCount = 0;
    swerve.stop();
  }
}
