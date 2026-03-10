package frc.robot.commands.hopper;

import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;

/** An example command that uses an example subsystem. */
public class LazyShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final int speed;
  private final double duration;
  private final double hoodAngle;

  private final Timer timer = new Timer();

  public LazyShooterCommand(
      HopperSubsystem hopperSubsystem, int speed, double hoodAngle, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    this.speed = speed;
    this.duration = duration;
    this.hoodAngle = hoodAngle;
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

    hopperSubsystem.setHood(hoodAngle);
    hopperSubsystem.setShooterVelocity(speed);
    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    if (timer.hasElapsed(0.5)) {
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    hopperSubsystem.stop();
    hopperSubsystem.setHood(0);
    timer.stop();
    timer.reset();
  }
}
