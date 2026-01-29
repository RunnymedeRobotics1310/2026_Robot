package ca.team1310.frc.robot.commands;

import ca.team1310.frc.robot.subsystems.ExampleSubsystem;
import ca.team1310.frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Lock the robot pose as long as this command is active (i.e. as long as the operator is pressing
 * the button.
 */
public class ExampleCommand extends Command {

  private final ExampleSubsystem exampleSubsystem;
  private final LightingSubsystem lightingSubsystem;

  public ExampleCommand(ExampleSubsystem exampleSubsystem, LightingSubsystem lightingSubsystem) {
    this.exampleSubsystem = exampleSubsystem;
    this.lightingSubsystem = lightingSubsystem;
    addRequirements(exampleSubsystem);
  }

  // runs once when the command starts
  @Override
  public void initialize() {
    System.out.println("Example Command init");

    System.out.println("Changing lights");
    lightingSubsystem.setBufferPattern(
        lightingSubsystem.ledTestingBufferOne, lightingSubsystem.yellowLEDPatern);
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
