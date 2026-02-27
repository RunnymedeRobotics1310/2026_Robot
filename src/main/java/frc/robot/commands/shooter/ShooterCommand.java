package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.CLOSE_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.FAR_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.FAR_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOT_HOOD_value;
import static frc.robot.Constants.ShooterConstants.SLOPE_VALUE_CLOSE;
import static frc.robot.Constants.ShooterConstants.SLOPE_VALUE_FAR;
import static frc.robot.Constants.ShooterConstants.SLOPE_VALUE_MID;
import static frc.robot.Constants.ShooterConstants.Y_INT_CLOSE;
import static frc.robot.Constants.ShooterConstants.Y_INT_FAR;
import static frc.robot.Constants.ShooterConstants.Y_INT_MID;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = swerveSubsystem.distanceToHub();
    log("Speed: " + shooterSubsystem.getShooterVelocity());
    shooting(distance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    shooterSubsystem.setHood(0);
    shooterSubsystem.stop();
    timer.stop();
    timer.reset();
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < MAX_SHOOTING_DISTANCE) {
      if (distanceMeters > FAR_SHOOTING_DISTANCE) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_FAR) + Y_INT_FAR;
        // log("Target speed: " + shooterSpeed);
      } else if (distanceMeters > MEDIUM_SHOOTING_DISTANCE) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_MID) + Y_INT_MID;
      } else {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_CLOSE) + Y_INT_CLOSE;
      }
    }
    return shooterSpeed;
  }

  public void shooting(double distance) {
    double shooterSpeed = calculateShootingSpeed(distance);
    shooterSubsystem.setShooterVelocity(shooterSpeed);

    if (swerveSubsystem.distanceToHub() > FAR_SHOOTING_DISTANCE) {
      shooterSubsystem.setHood(FAR_SHOOT_HOOD_VALUE);
    } else if (swerveSubsystem.distanceToHub() > MEDIUM_SHOOTING_DISTANCE) {
      shooterSubsystem.setHood(MEDIUM_SHOOT_HOOD_value);
    } else {
      shooterSubsystem.setHood(CLOSE_SHOOT_HOOD_VALUE);
    }

    // Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed) < 10

    if (timer.hasElapsed(1.75)) {
      shooterSubsystem.setKickerSpeed(-0.7);
    }
    if (timer.hasElapsed(2.0)) {
      shooterSubsystem.setKickerSpeed(0.0);
      timer.reset();
      timer.stop();
    }
  }
}
