package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "rotate",
    category = "drive",
    description = "Rotate in place to a target heading")
public class RotateToHeadingCommand extends LoggingCommand {

  private static final double TOLERANCE_DEGREES = 2.0;
  private static final double STABLE_TIME_SECONDS = 0.06;
  private static final double MAX_OMEGA_RAD_PER_SEC = 0.80;
  private static final double HEADING_HOLD_ENTER_DEGREES = 2.0;
  private static final double HEADING_HOLD_EXIT_DEGREES = 4.0;

  private final SwerveSubsystem swerve;
  private final double heading;
  private final double timeoutSeconds;

  private double allianceHeading;
  private double stableSinceSeconds = -1;
  private boolean headingHoldActive = false;

  public RotateToHeadingCommand(
      SwerveSubsystem swerve,
      @ConfigParam(value = "headingDegrees", unit = "deg", description = "Target heading")
          double heading,
      @ConfigParam(
              value = "timeoutSeconds",
              unit = "s",
              min = 0,
              max = 15,
              description = "Safety timeout")
          double timeoutSeconds) {
    this.swerve = swerve;
    this.heading = heading;
    this.timeoutSeconds = timeoutSeconds;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
    }
    allianceHeading = heading + headingOffset;
    stableSinceSeconds = -1;
    headingHoldActive = false;

    logCommandStart("hdg=" + allianceHeading + " timeout=" + timeoutSeconds);
  }

  @Override
  public void execute() {
    double omega = computeHeadingOmega();
    swerve.driveFieldOriented(0, 0, omega);
  }

  @Override
  public boolean isFinished() {
    double currentHeading = swerve.getYaw();
    double error = Math.abs(swerve.getHeadingErrorDegrees(allianceHeading));

    boolean stableNow = error <= TOLERANCE_DEGREES;
    if (stableNow) {
      if (stableSinceSeconds < 0) {
        stableSinceSeconds = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - stableSinceSeconds >= STABLE_TIME_SECONDS) {
        setFinishReason("Heading settled: " + format(currentHeading) + " deg err=" + format(error));
        return true;
      }
    } else {
      stableSinceSeconds = -1;
    }
    if (timeoutSeconds > 0 && hasElapsed(timeoutSeconds)) {
      setFinishReason("Timeout after " + timeoutSeconds + "s");
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    logCommandEnd(interrupted);
  }

  private double computeHeadingOmega() {
    double error = Math.abs(swerve.getHeadingErrorDegrees(allianceHeading));

    if (headingHoldActive) {
      if (error <= HEADING_HOLD_EXIT_DEGREES) {
        return 0;
      }
      headingHoldActive = false;
    }

    if (error <= HEADING_HOLD_ENTER_DEGREES) {
      headingHoldActive = true;
      return 0;
    }

    return swerve.computeOmega(allianceHeading, MAX_OMEGA_RAD_PER_SEC);
  }
}
