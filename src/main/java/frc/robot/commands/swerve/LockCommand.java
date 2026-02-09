package frc.robot.commands.swerve;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LockCommand extends LoggingCommand {

  SwerveSubsystem swerve;

  LockCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {

    swerve.lock();
  }
}
