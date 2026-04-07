package frc.robot.commands.swerve;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "face_hub", category = "drive", description = "Rotate to face the hub")
public class FaceHubCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Translation2d velocity;

  public FaceHubCommand(SwerveSubsystem swerve) {
    this(swerve, new Translation2d());
  }

  public FaceHubCommand(SwerveSubsystem swerve, Translation2d velocity) {
    this.swerve = swerve;
    this.velocity = velocity;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    double hubAngle = swerve.calculateHubAngleDegForVelocity(velocity);
    double omega = swerve.computeOmega(hubAngle);
    log("omega: " + omega);
    swerve.driveFieldOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {
    return SwerveUtils.isCloseEnough(swerve.getYaw(), swerve.getHubAngleDeg(), 3);
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
    log("stopping!!!!!");
  }
}
