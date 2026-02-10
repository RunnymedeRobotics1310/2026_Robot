package frc.robot.commands.swerve;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToLeftTowerCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 50;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private int tagId = -1;
  private int noDataCount = 0;


  public DriveToLeftTowerCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
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
    } else {
      tagId = 31;
    }
    tagId = 31;
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

      double theta = 0;
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
    if (Math.abs(tX+20) > 20) {
      vX = 0;
    } else {
      vX = -0.35;
    }
    vY = 0.07 * (tX+20);

    double theta = 0;
    double omega = swerve.computeOmega(theta);

    swerve.driveRobotOriented(vX, vY, omega);
  }

  @Override
  public boolean isFinished() {
    double theta = 0;

    if (noDataCount > MAX_NO_DATA_COUNT_CYCLES && Math.abs(swerve.getYaw() - theta) < 5) {
      log("Finishing - no vision data for " + noDataCount + " cycles");
      return true;
    }

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
