package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveFieldOrientedCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private double x;
  private double y;
  private final double heading;
  private double allianceHeading;

  public DriveFieldOrientedCommand(SwerveSubsystem swerve, double x, double y, double heading) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.heading = heading;
  }

  @Override
  public void initialize() {
    logCommandStart();

    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
      x = -x;
      y = -y;
    }

    allianceHeading = heading + headingOffset;
  }

  @Override
  public void execute() {
    swerve.driveFieldOriented(x, y, swerve.computeOmega(allianceHeading));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    logCommandEnd(interrupted);
  }
}
