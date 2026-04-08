package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "drive_field_oriented",
    category = "drive",
    description = "Drive field-oriented at a velocity and heading")
public class DriveAimFieldOrientedCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private double x;
  private double y;

  public DriveAimFieldOrientedCommand(SwerveSubsystem swerve, double x, double y) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
  }

  @Override
  public void initialize() {
    logCommandStart();

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      x = -x;
      y = -y;
    }
  }

  @Override
  public void execute() {
    swerve.driveFieldOriented(x, y, swerve.computeOmega(swerve.getHubAngleDeg()));
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
