package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "face_hub", category = "drive", description = "Rotate to face the hub")
public class FaceHubCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;

  public FaceHubCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    Rotation2d hubAngle = swerve.angleToShootTowards();
    double omega = swerve.computeOmega(hubAngle.getDegrees());
    log("omega: " + omega);
    swerve.driveFieldOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {
    double error = Math.abs(swerve.angleToShootTowards().getDegrees() - swerve.getYaw());
    log("Error: " + error);
    return error <= 3;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
    log("stopping!!!!!");
  }
}
