// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;

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

    if (climb.isAutoClimbed && DriverStation.isTeleopEnabled()) {
      climb.setClimbSpeed(1);
      if (climb.getClimbPosition() >= Constants.ClimbConstants.MAX_CLIMB_POSITION) {
        climb.isAutoClimbed = false;
        climb.setClimbSpeed(0);
      }
    } else if (input.isIntakeDoingStuff() && !climb.isClimbDown()) climb.setClimbSpeed(-1);
    else climb.setClimbSpeed(input.getOperatorController().getLeftY());
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
    climb.stop();
    if (interrupted && DriverStation.isTeleopEnabled()) climb.isAutoClimbed = false;
  }
}
