package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class CloseShootCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public CloseShootCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
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
    shooterSubsystem.setHood(0);
    shooterSubsystem.stop();
    timer.stop();
    timer.reset();
  }

  public void shootClose() {
    shooterSubsystem.setHood(0.0);
    int targetspeed = 2900;
    shooterSubsystem.setShooterVelocity(targetspeed);
    if (shooterSubsystem.getShooterVelocity() > 2850) {
      shooterSubsystem.setKickerSpeed(0.7);
    } else {
      shooterSubsystem.setKickerSpeed(0.0);
    }
  }
}
