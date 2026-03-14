package frc.robot.commands.auto;

import static frc.robot.Constants.ClimbConstants.MAX_CLIMB_POSITION;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class AutoClimbCommand extends LoggingCommand {

  HopperSubsystem hopper;
  ClimbSubsystem climb;

  private final boolean climbGoingUp;

  AutoClimbCommand(ClimbSubsystem climb, HopperSubsystem hopper, boolean climbGoingUp) {
    addRequirements(climb);
    addRequirements(hopper);

    this.hopper = hopper;
    this.climb = climb;
    this.climbGoingUp = climbGoingUp;
  }

  @Override
  public void initialize() {
    logCommandStart();
    hopper.setDoorSetpoint(0);
  }

  @Override
  public void execute() {
    hopper.setDoorSetpoint(0);
    if (climbGoingUp) {
      climb.setClimbSpeed(1);
    } else {
      climb.setClimbSpeed(-1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }

  @Override
  public boolean isFinished() {
    if (climbGoingUp && climb.getPos() >= MAX_CLIMB_POSITION) {
      return true;
    }
    if (!climbGoingUp && climb.getPos() <= 0) {
      return true;
    }
    return false;
  }
}
