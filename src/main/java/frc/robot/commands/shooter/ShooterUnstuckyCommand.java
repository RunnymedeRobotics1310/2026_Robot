package frc.robot.commands.shooter;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterUnstuckyCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  public ShooterUnstuckyCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    unstuckShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    shooterSubsystem.stop();
  }

  public void unstuckShooter() {
    shooterSubsystem.setKickerSpeed(1);
  }
}
