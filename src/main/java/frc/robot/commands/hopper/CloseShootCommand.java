package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;

/** An example command that uses an example subsystem. */
public class CloseShootCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param hopperSubsystem The subsystem used by this command.
   */
  public CloseShootCommand(HopperSubsystem hopperSubsystem) {
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootClose();
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

  public void shootClose() {
    hopperSubsystem.setHood(0.0);
    int targetspeed = 3400;
    hopperSubsystem.setShooterVelocity(targetspeed);
    if (hopperSubsystem.isShooterAtSpeed()) {
      hopperSubsystem.setKickerSpeed(0.7);
    } else {
      hopperSubsystem.setKickerSpeed(0.0);
    }
  }
}
