package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final Timer timer = new Timer();
  private boolean firstShoot;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(HopperSubsystem hopperSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    firstShoot = true;
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
    log("Speed: " + hopperSubsystem.getShooterVelocity());
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
    hopperSubsystem.setHood(0);
    hopperSubsystem.stop();
    timer.stop();
    timer.reset();
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < MAX_SHOOTING_DISTANCE) {
      if (distanceMeters >= SUPER_FAR_SHOOTING_DISTANCE) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_SUPER_FAR) + Y_INT_SUPER_FAR;
      } else if (distanceMeters >= MEDIUM_SHOOTING_DISTANCE) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_MID) + Y_INT_MID;
      } else {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_CLOSE) + Y_INT_CLOSE;
      }
    }
    return shooterSpeed;
  }

  public void shooting(double distance) {
    double targetSpeed = calculateShootingSpeed(distance);
    hopperSubsystem.setShooterVelocity(targetSpeed);
    double currentVelocity = hopperSubsystem.getShooterVelocity();
    boolean atSpeed = (targetSpeed - currentVelocity) < ACCPETED_SHOOTER_ERROR;

    if (distance < MAX_SHOOTING_DISTANCE) {
      if (distance >= SUPER_FAR_SHOOTING_DISTANCE) {
        hopperSubsystem.setHood(SUPER_FAR_SHOOT_HOOD_VALUE);
      } else if (distance >= MEDIUM_SHOOTING_DISTANCE) {
        hopperSubsystem.setHood(MEDIUM_SHOOT_HOOD_VALUE);

      } else {
        hopperSubsystem.setHood(CLOSE_SHOOT_HOOD_VALUE);
      }
    }

    if (atSpeed) {
      firstShoot = true;
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
      timer.reset();

    } else if (timer.get() < 0.8 && firstShoot) {
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else {
      hopperSubsystem.setKickerSpeed(0);
    }

    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
  }
}