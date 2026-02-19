// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import static frc.robot.operatorInput.OperatorInput.Axis.Y;
import static frc.robot.operatorInput.OperatorInput.Stick.RIGHT;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends LoggingCommand {

  private final ClimbSubsystem climb;
  private final OperatorInput input;

  public ClimbCommand(ClimbSubsystem climb, OperatorInput input) {
    super();
    this.climb = climb;
    this.input = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setClimbSpeed(input.getDriverControllerAxis(RIGHT, Y));
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
  }
}
