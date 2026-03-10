package frc.robot.commands.hopper;

import static frc.robot.Constants.ShooterConstants.*;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final Timer timer = new Timer();
  private boolean firstShot = false;

  public ShooterCommand(HopperSubsystem hopperSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    timer.start();
    firstShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooting();
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

  public void shooting() {
    // shooter
    double distance = swerveSubsystem.distanceToHub();
    double targetSpeed = hopperSubsystem.calculateShootingSpeed(distance);
    hopperSubsystem.setShooterVelocity(targetSpeed);

    // agitator
    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);

    // hood
    if (distance < MAX_SHOOTING_DISTANCE) {
      if (distance >= SUPER_FAR_SHOOTING_DISTANCE) {
        hopperSubsystem.setHood(SUPER_FAR_SHOOT_HOOD_VALUE);
      } else if (distance >= MEDIUM_SHOOTING_DISTANCE) {
        hopperSubsystem.setHood(MEDIUM_SHOOT_HOOD_VALUE);
      }
    } else {
      hopperSubsystem.setHood(CLOSE_SHOOT_HOOD_VALUE);
    }

    // kicker
    double currentVelocity = hopperSubsystem.getShooterVelocity();
    boolean atSpeed = Math.abs(targetSpeed - currentVelocity) < ACCPETED_SHOOTER_ERROR;
    boolean facingHub =
        SwerveUtils.isCloseEnough(
            swerveSubsystem.angleToHub().getDegrees(), swerveSubsystem.getYaw(), 5);

    if ((atSpeed || firstShot) && facingHub) {
      firstShot = true;
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else {
      hopperSubsystem.setKickerSpeed(0);
    }
  }
}
