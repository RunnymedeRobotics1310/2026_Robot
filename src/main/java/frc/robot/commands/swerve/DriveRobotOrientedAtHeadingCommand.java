package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "drive_velocity",
    category = "drive",
    description = "Drive robot-oriented at a heading until interrupted")
public class DriveRobotOrientedAtHeadingCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final double x;
  private final double y;
  private final double heading;
  private double allianceHeading;

  public DriveRobotOrientedAtHeadingCommand(
      SwerveSubsystem swerve,
      @ConfigParam(value = "vxMPS", unit = "m/s", description = "Forward velocity") double x,
      @ConfigParam(value = "vyMPS", unit = "m/s", description = "Left velocity") double y,
      @ConfigParam(value = "headingDegrees", unit = "deg", description = "Robot heading to hold")
          double heading) {
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
    }

    allianceHeading = heading + headingOffset;
  }

  @Override
  public void execute() {
    swerve.driveRobotOriented(x, y, swerve.computeOmega(allianceHeading));
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
