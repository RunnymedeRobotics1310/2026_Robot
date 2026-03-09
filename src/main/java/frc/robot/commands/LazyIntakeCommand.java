package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.IntakeConstants.INTAKE_DOOR_ANGLE;

public class LazyIntakeCommand extends LoggingCommand {

  private final IntakeSubsystem intakeSubsystem;

  public LazyIntakeCommand(
      IntakeSubsystem intake) {
    super();
    intakeSubsystem = intake;
    addRequirements(intake);
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

    intakeSubsystem.setRollerSpeeds(-1, -0.8);
    intakeSubsystem.setDoorSetpoint(INTAKE_DOOR_ANGLE);
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
    intakeSubsystem.setRollerSpeeds(0, 0);
    intakeSubsystem.setDoorSetpoint(0);
  }
}
