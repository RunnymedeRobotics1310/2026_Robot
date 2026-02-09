package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveRobotOrientedAtHeading extends LoggingCommand {

  private double x;
  private double y;
  private double heading;
  private SwerveSubsystem swerve;
  private double allianceHeading;

  public DriveRobotOrientedAtHeading(SwerveSubsystem swerve, double x, double y, double heading) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.heading = heading;
  }

  @Override
  public void initialize() {
    logCommandStart();

    double allianceOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceOffset = 180;
    }

    allianceHeading = heading + allianceOffset;
  }

  @Override
  public void execute() {
    swerve.driveRobotOriented(x, y, swerve.computeOmega(allianceHeading));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    logCommandEnd(interrupted);
  }
}
