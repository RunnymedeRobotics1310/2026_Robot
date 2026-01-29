package ca.team1310.frc.robot.commands;

import ca.team1310.frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Lock the robot pose as long as this command is active (i.e. as long as the operator is pressing
 * the button.
 */
public class ExampleCommand extends Command {

  private final ExampleSubsystem exampleSubsystem;

  public ExampleCommand(ExampleSubsystem exampleSubsystem) {
    this.exampleSubsystem = exampleSubsystem;
    addRequirements(exampleSubsystem);
  }

  // runs once when the command starts
  @Override
  public void initialize() {
    System.out.println("Example Command init");
  }

  // Runs once every robot period
  @Override
  public void execute() {
    System.out.println("Example Command execute");
    exampleSubsystem.exampleMethod();
  }

  // when the command should finish (runs every robot period)
  @Override
  public boolean isFinished() {
    return true;
  }

  // Runs once when the command ends
  @Override
  public void end(boolean interrupted) {
    System.out.println("Example Command ended");
  }
}
