package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.INTAKE_DOOR_ANGLE;

import frc.robot.subsystems.HopperSubsystem;

public class LazyIntakeCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  public LazyIntakeCommand(HopperSubsystem hopper) {
    super();
    hopperSubsystem = hopper;
    addRequirements(hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // door go out
    // spin motors
    // only when you press a button

    hopperSubsystem.setRollerSpeeds(-1, -0.8);
    hopperSubsystem.setDoorSetpoint(INTAKE_DOOR_ANGLE);
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
    hopperSubsystem.setRollerSpeeds(0, 0);
    hopperSubsystem.setDoorSetpoint(0);
  }
}
