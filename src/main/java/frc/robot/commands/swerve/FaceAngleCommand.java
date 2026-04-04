package frc.robot.commands.swerve;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "face_hub", category = "drive", description = "Rotate to face")
public class FaceAngleCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Rotation2d angle;
  private Rotation2d allianceAngle;

  public FaceAngleCommand(SwerveSubsystem swerve, Rotation2d angle) {
    this.swerve = swerve;
    this.angle = angle;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceAngle = angle.plus(Rotation2d.fromDegrees(180));
    } else {
      allianceAngle = angle;
    }
  }

  @Override
  public void execute() {
    double omega = swerve.computeOmega(allianceAngle.getDegrees());
    log("omega: " + omega);
    swerve.driveFieldOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {
    return SwerveUtils.isCloseEnough(allianceAngle.getDegrees(), swerve.getYaw(), 5);
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
    log("stopping!!!!!");
  }
}
