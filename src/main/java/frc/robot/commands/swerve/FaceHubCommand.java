package frc.robot.commands.swerve;

import ca.team1310.swerve.utils.SwerveUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "face_hub", category = "drive", description = "Rotate to face the hub")
public class FaceHubCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final boolean noFinish;

  public FaceHubCommand(SwerveSubsystem swerve) {
    this(swerve, false);
  }

  public FaceHubCommand(SwerveSubsystem swerve, boolean noFinish) {
    this.swerve = swerve;
    this.noFinish = noFinish;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    double hubAngle = swerve.getHubAngleDeg();
    double omega = swerve.computeOmega(hubAngle);
    swerve.driveFieldOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {
    if (noFinish) return false;
    return SwerveUtils.isCloseEnough(swerve.getYaw(), swerve.getHubAngleDeg(), 3);
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
    log("facing hub");
  }
}
